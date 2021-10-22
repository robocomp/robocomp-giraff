/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <cppitertools/enumerate.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    try
    {
        RoboCompCommonBehavior::Parameter par = params.at("inverted_tilt");
        if(par.value == "true" or par.value == "True")
            inverted_tilt = -1;
        else inverted_tilt = 1;
    }
    catch(const std::exception &e) { qFatal("Error reading config params"); }
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;

    try{ jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{0.0001, 1 });}  // radians. 0 vertical
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    // Load YOLO network
    std::string modelConfiguration = "yolov3/yolov3.cfg";  //yolov4 is not supported in this version of OPenCV
    std::string modelWeights = "yolov3/yolov3.weights";
    std::string classesFile = "yolov3/coco.names";
    std::ifstream ifs(classesFile.c_str());
    std::string line;
    while (getline(ifs, line)) classes.push_back(line);
    net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

//    // Create an instance of Facemark
//    faceDetector.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt2.xml");
//    facemark = cv::face::FacemarkLBF::create();
//    // Load landmark detector
//    facemark->loadModel("opencv_face/lbfmodel.yaml");

    connect(&timer, SIGNAL(timeout()), this, SLOT(compute_L1()));
    this->Period = period;
    if(this->startup_check_flag)
        this->startup_check();
    else
        timer.start(Period);
}

////////////////////////////////////////////////////
// THREE LEVELS ARCH. EACH LEVEL IS A STATE MACHINE
///////////////////////////////////////////////////

// FIST LEVEL. read sensors and use classifiers to create dynamic control loops
// that keep perceived objects (person parts) centered (in focus). Output presence state and pose
void SpecificWorker::compute_L1()
{
    const float XMARK = 10;  // number or hits before getting 0.7
    const float YMARK = 0.7; // threshold value for next level
    static int cont = 0;
    const double s = -XMARK/(log(1.0/YMARK - 1.0));
    auto integrator = [s](double x){return 1.0/(1.0 + exp(-x/s));};

    const auto &[body_o, face_o] = read_image();
    cont = std::clamp(cont, -20, 20);
    qInfo() << l1_map.at(l1_state) << dyn_state;
    switch(l1_state)
    {
        case L1_State::SEARCHING:
            if(face_o.has_value())
                l1_state = L1_State::FACE_DETECTED;
            else if(body_o.has_value())
                l1_state = L1_State::BODY_DETECTED;
            dyn_state = integrator(cont--);
            // rotate to look for the person
            break;

        case L1_State::BODY_DETECTED:
            if(not body_o.has_value())
            {
                l1_state = L1_State::SEARCHING;
                return;
            }
            move_tablet(body_o, face_o);
            move_base(body_o, face_o);
            dyn_state = integrator(cont++);
            break;
        case L1_State::FACE_DETECTED:
            if(not face_o.has_value())
            {
                emotionalmotor_proxy->isanybodythere(false);
                l1_state = L1_State::SEARCHING;

                return;
            }
            emotionalmotor_proxy->isanybodythere(true);
            move_tablet(body_o, face_o);
            move_base(body_o, face_o);
            move_eyes(face_o);
            dyn_state = integrator(cont++);
            break;
        case L1_State::EYES_DETECTED:
            break;
        case L1_State::HANDS_DETECTED:
            break;
    }
}
void SpecificWorker::compute_L2()
{
    // SECOND LEVEL: belief function to create and maintain a represented person
    // we need here to create, maintain and destroy (when needed) a "represented person" that exists because
    // enough evidence supports it. We want evidence coming from the control loops, not just the classifiers
    // The logic here is, when new evidence (presence state and pose) gets in
    // if there is no person, create it. It must have certain internal structure.
    // from here, synthesize the evidence that should be coming and is relevant to the task
    // if evidence comes, check if it matches. If it does, update representation (and dynamic model of it)
    // if it does not match, note as counter evidence
    // it no evidence comes, note as counter evidence
    // if enough counter evidence has accumulated, destroy the representation

    switch (l2_state)
    {
        case L2_State::EXPECTING:
            if(dyn_state > 0.7)
            {
                // create person
                l2_state = L2_State::PERSON;
            }
            break;
        case L2_State::PERSON:
            if(dyn_state < 0.7)
            {
                // delete person. Maybe push into memory
                l2_state = L2_State::EXPECTING;
            }
            // update person
            // project where person is going to be
            // plan perception annd send downwards anticipated proposal
            break;
    }
}
void SpecificWorker::compute_L3()
{
// THIRD LEVEL.  Mission specific control program to decide what to do next based on the represented person
//    switch(state)
//    {
//        case State::WAITING:
//            // if person detected move to PERSON_DETECTED
//            break;
//        case State::PERSON_DETECTED:
//            break;
//        case State::READY_TO_INTERACT:
//            // Wait for a signal or order and move to FOLLOW
//            break;
//        case State::START_FOLLOWING:
//            // check person is moving away
//            break;
//        case State::FOLLOWING:
//            // distance control
//            break;
//        case State::STOP:
//            // person has stopped
//            break;
//    };
}
void SpecificWorker::compute(){};
////////////////////////////////////////////////////////////////////
SpecificWorker::DetectRes SpecificWorker::read_image()
{
    try
    {
        //RoboCompCameraRGBDSimple::TImage top_img = camerargbdsimple_proxy->getImage("camera_tablet");
        RoboCompCameraSimple::TImage rgb = camerasimple_proxy->getImage();

        // const auto &top_img = rgbd.image;
        // const auto &top_depth = rgbd.depth;
        const int width = 300;
        const int height = 300;
        if (rgb.width != 0 and rgb.height != 0)
        {
            cv::Mat img;
            if (rgb.compressed)
            {
                auto img_uncomp = cv::imdecode(rgb.image, -1);
                img = cv::Mat(rgb.height, rgb.width, CV_8UC3, &img_uncomp.data);

            }
            else
                img = cv::Mat(rgb.height, rgb.width, CV_8UC3, &rgb.image[0]);

            cv::Mat n_img;
            cv::resize(img, n_img, cv::Size(width, height));

            DetectRes ret{{},{}};

            // face
            auto rectangles = face_detector.detect_face_rectangles(n_img);
            if (not rectangles.empty())
            {
                std::cout << "if" << std::endl;
                auto r = rectangles.front();
                cv::rectangle(n_img, r, cv::Scalar(0, 105, 205), 3);
                QRect rect = QRect(r.x, r.y, r.width, r.height);
                //float k = depth.at<float>(rect.center().x(), rect.center().y()) * 1000;  //mmm
                float k = 0;
                std::get<1>(ret) = {(width/2)-rect.center().x(), (height/2)-rect.center().y(), k};

//                std::vector<cv::Rect> faces;
//                // Convert frame to grayscale because it requires grayscale image.
//                cv::Mat gray;
//                cv::cvtColor(n_img, gray, cv::COLOR_BGR2GRAY);
//                // Detect faces
//                faceDetector.detectMultiScale(gray, faces);
//                // Landmarks for one face is a vector of points
//                // There can be more than one face in the image. Hence, we use a vector of vector of points.
//                std::vector< vector<cv::Point2f> > landmarks;
//                // Run landmark detector
//                bool success = facemark->fit(n_img,faces,landmarks);
//                if(success)
//                    qInfo() << __FUNCTION__ << "Landmarks" << landmarks.size();
            }
            else //body
            {
                // YOLO
                auto people = yolo_detector(n_img);
                if (not people.empty())
                {
                    auto r = people.front();
                    cv::rectangle(n_img, r, cv::Scalar(0, 0, 255), 3);
                    QRect rect = QRect(r.x, r.y, r.width, r.height);
                    // float k = depth.at<float>(rect.center().x(), rect.center().y()) * 1000;  //mmm
                    float k = 0;
                    std::get<0>(ret) = {(width/2) - rect.center().x(), (height/2) - rect.center().y(), k};

                }
            }
            // show image
            cv::cvtColor(n_img, n_img, cv::COLOR_BGR2RGB);
            cv::imshow("Camera tablet", n_img);
            cv::waitKey(1);
            return ret;
        }
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    qWarning() << "Empty image or comms problem";
    return {};
}
void SpecificWorker::move_tablet(std::optional<std::tuple<int,int,int>> body_o, std::optional<std::tuple<int,int,int>> face_o)
{
    float tilt = 0.7;
    float pos;
    if(body_o.has_value())
    {
        auto [_, body_y_error, __] = body_o.value();
        const float delta = 0.1;
        tilt = inverted_tilt * (delta / 100) * body_y_error;  // map from -100,100 to -0.1,0.1 rads
        pos = jointmotorsimple_proxy->getMotorState("tablet_joint").pos;
        //qInfo() << __FUNCTION__ << "BODY: pos" << pos << "error" << body_y_error << "tilt" << tilt << pos - tilt;
        cout << "Move tablet body" << endl;
        try { jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{pos - tilt, 1}); }  // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to join motor " << std::endl; }
    }
    else if( face_o.has_value())
    {
        auto [_, face_y_error, __] = face_o.value();
        const float delta = 0.1;
        tilt = inverted_tilt * (delta / 100) * face_y_error;  // map from -100,100 to -0.1,0.1 rads
        pos = jointmotorsimple_proxy->getMotorState("tablet_joint").pos;
        //qInfo() << __FUNCTION__ << "FACE: pos" << pos << "error" << face_y_error << "tilt" << tilt << pos - tilt;
        cout << "Move tablet face" << endl;
        try { jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{pos - tilt, 1}); }  // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    }

    if (abs(tilt) > 0.1 )
    {
        try { jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{pos - tilt, 1}); }  // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    }

    /*else{
        try { jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{pos - tilt, 1}); }  // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    }*/
}
void SpecificWorker::move_base(std::optional<std::tuple<int,int,int>> body_o, std::optional<std::tuple<int,int,int>> face_o)
{
    // rotate base
    const float gain = 0.3;
    float advance = 0.0;
    float rot = 0.0;
    if(body_o.has_value())
    {
        auto &[body_x_error, _, body_dist] = body_o.value();
        rot = -(2.f / 100) * body_x_error;
        cout << "/////// rot:" << rot << endl;
        if(body_dist > 0 and body_dist < 800)
            advance = -(400.0 / 800.0) * body_dist;

    }
    else if(face_o.has_value())
    {
        auto &[face_x_error, _, face_dist] = face_o.value();
        rot = -(2.f / 100) * face_x_error;
        if(face_dist > 0 and face_dist < 800)
            advance = -(400.0 / 800.0) * face_dist;
    }

    if (abs(rot) > 0.5 )
    {
        try { differentialrobot_proxy->setSpeedBase(advance, gain * rot); }
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    }

    else{
        try { differentialrobot_proxy->setSpeedBase(0.0, 0.0); }
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    }
}
void SpecificWorker::move_eyes(std::optional<std::tuple<int,int,int>> face_o)
{
    if( face_o.has_value())
    {

        auto [face_x_error, face_y_error, __] = face_o.value();
        const float delta = 0.1;
        const float tilt_x = (delta / 10) * face_x_error;  // map from -100,100 to -0.1,0.1 rads
        const float tilt_y = -(delta / 10) * face_y_error;  // map from -100,100 to -0.1,0.1 rads
        cout << tilt_x << std::endl;
        cout << tilt_y << std::endl;
        //qInfo() << __FUNCTION__ << "FACE: pos" << pos << "error" << face_y_error << "tilt" << tilt << pos - tilt;
        try { emotionalmotor_proxy->pupposition(tilt_x,tilt_y); }  // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    }
}
std::vector<cv::Rect> SpecificWorker::yolo_detector(cv::Mat &frame)
{
    auto blob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(), true,false);
    net.setInput(blob);
    vector<cv::Mat> outs;
    net.forward(outs, get_outputs_names(net));

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);

            if (confidence > confThreshold and classes[classIdPoint.x] == "person")
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
                cout << " // " << classes[classIdPoint.x] << endl;

            }
        }
    }
    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    std::vector<cv::Rect> new_boxes;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        new_boxes.push_back(box);
        //drawPred(classIds[idx], confidences[idx], box.x, box.y, box.x + box.width, box.y + box.height, frame);
    }
    return new_boxes;
}
std::vector<cv::String> SpecificWorker::get_outputs_names(const cv::dnn::Net &net)
{
    static std::vector<cv::String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<cv::String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompEmotionalMotor you can call this methods:
// this->emotionalmotor_proxy->expressAnger(...)
// this->emotionalmotor_proxy->expressDisgust(...)
// this->emotionalmotor_proxy->expressFear(...)
// this->emotionalmotor_proxy->expressJoy(...)
// this->emotionalmotor_proxy->expressSadness(...)
// this->emotionalmotor_proxy->expressSurprise(...)




/////// HOG

// body
// vector<cv::Rect> found;
// vector<double> weights;
// cv::Mat n_img_bw;
// cv::cvtColor(n_img, n_img_bw, cv::COLOR_BGR2GRAY);
// hog.detectMultiScale(n_img_bw, found, weights);


/**************************************/
// From the RoboCompCameraSimple you can call this methods:
// this->camerasimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompEmotionalMotor you can call this methods:
// this->emotionalmotor_proxy->expressAnger(...)
// this->emotionalmotor_proxy->expressDisgust(...)
// this->emotionalmotor_proxy->expressFear(...)
// this->emotionalmotor_proxy->expressJoy(...)
// this->emotionalmotor_proxy->expressSadness(...)
// this->emotionalmotor_proxy->expressSurprise(...)

/**************************************/
// From the RoboCompJointMotorSimple you can call this methods:
// this->jointmotorsimple_proxy->getMotorParams(...)
// this->jointmotorsimple_proxy->getMotorState(...)
// this->jointmotorsimple_proxy->setPosition(...)
// this->jointmotorsimple_proxy->setVelocity(...)
// this->jointmotorsimple_proxy->setZeroPos(...)

/**************************************/
// From the RoboCompJointMotorSimple you can use this types:
// RoboCompJointMotorSimple::MotorState
// RoboCompJointMotorSimple::MotorParams
// RoboCompJointMotorSimple::MotorGoalPosition
// RoboCompJointMotorSimple::MotorGoalVelocity

