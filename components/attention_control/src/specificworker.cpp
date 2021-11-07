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
    connect(&timer_l2, SIGNAL(timeout()), this, SLOT(compute_L2()));
    connect(&timer_l3, SIGNAL(timeout()), this, SLOT(compute_L3()));
    this->Period = 80;
    int Period_L2 = 150;
    int Period_L3 = 220;
    if(this->startup_check_flag)
        this->startup_check();
    else
    {
        timer.start(Period);
        timer_l2.start(Period_L2);
        timer_l3.start(Period_L3);
    }
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
    switch(l1_state)
    {
        case L1_State::SEARCHING:
            if (face_o.has_value())
            {
                auto &[_, __, d] = face_o.value();
                this->l1_person.pos = Eigen::Vector2f(0, d);
                this->l1_person.looking_at = Parts::FACE;
                l1_state = L1_State::FACE_DETECTED;
                return;
            } else if (body_o.has_value())
            {
                auto &[_, __, d] = body_o.value();
                l1_state = L1_State::BODY_DETECTED;
                this->l1_person.pos = Eigen::Vector2f(0, d);
                this->l1_person.looking_at = Parts::BODY;
//                try{ differentialrobot_proxy->setSpeedBase(0, 0); robot.current_rot_speed = 0;  robot.current_adv_speed=0.0;}
//                catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to differential robot" << std::endl; }
                return;
            }
            this->l1_person.looking_at = Parts::NONE;
            this->l1_person.dyn_state = integrator(cont--);
            // rotate to look for the person
            if( robot.current_rot_speed > 0)
                try{ differentialrobot_proxy->setSpeedBase(0, 0.5); robot.current_rot_speed = 0.5;  robot.current_adv_speed=0.0;}
                catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to differential robot" << std::endl; }
            else
                try{ differentialrobot_proxy->setSpeedBase(0, -0.5); robot.current_rot_speed = -0.5;  robot.current_adv_speed=0.0;}
                catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to differential robot" << std::endl; }
            break;

        case L1_State::BODY_DETECTED:
            if(not body_o.has_value())
            {
                l1_state = L1_State::SEARCHING;
                return;
            }
            move_tablet(body_o, face_o);
            move_base(body_o, face_o);
            this->l1_person.pos = Eigen::Vector2f(0, std::get<2>(body_o.value()));
            this->l1_person.dyn_state = integrator(cont++);
            break;

        case L1_State::FACE_DETECTED:
            if(not face_o.has_value())
            {
                //try{ emotionalmotor_proxy->isanybodythere(false);}
                //catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to emotional motor " << std::endl; }
                l1_state = L1_State::SEARCHING;
                return;
            }
            //try{ emotionalmotor_proxy->isanybodythere(true);}
            //catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to emotional motor " << std::endl; }
            this->l1_person.pos = Eigen::Vector2f(0, std::get<2>(face_o.value()));
            move_tablet(body_o, face_o);
            move_base(body_o, face_o);
            //move_eyes(face_o);
            this->l1_person.dyn_state = integrator(cont++);
            break;
        case L1_State::EYES_DETECTED:
            break;
        case L1_State::HANDS_DETECTED:
            break;
    }
   this->l1_person.print();
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
            if(this->l1_person.dyn_state > 0.7)
            {
                // create person
                L2_Person p;
                p.pos =  Eigen::Vector2f(0,0);
                p.vel = Eigen::Vector2f(0,0);
                p.looking_at = l1_person.looking_at;
                p.height = 160;
                p.name = "Bill";
                p.orientation = 0;
                p.angular_spped = 0;
                p.creation_time = std::chrono::system_clock::now();
                l2_people.push_back(p);
                l2_state = L2_State::PERSON;
            }
            break;
        case L2_State::PERSON:
            if(this->l1_person.dyn_state < 0.7)
            {
                // delete person. Maybe push into memory
                l2_people.clear();
                l2_state = L2_State::EXPECTING;
            }
            // update person
            L2_Person &p = l2_people[0];
            if(l1_person.pos.y() > 0 )  // don't update if bad depth reading
                p.pos = l1_person.pos;
            p.looking_at = l1_person.looking_at;
            // downward flow
            // project where person is going to be
            // plan perception annd send downwards anticipated proposal
            l1_person.current_action = p.current_action;
            break;
    }
//    if(not l2_people.empty())
//        l2_people.front().print();
}
void SpecificWorker::compute_L3()
{
// THIRD LEVEL.  Mission specific control program to decide what to do next based on the represented person
    static std::chrono::system_clock::time_point ready_start;
    static std::chrono::system_clock::time_point searching_initial_time;
    switch(l3_state)
    {
        case L3_State::WAITING:
            // if person detected move to PERSON_DETECTED
            if( not l2_people.empty()
                and l2_people.front().looking_at == Parts::FACE
                and l2_people.front().pos.norm() < 800)
            {
               l3_state = L3_State::READY_TO_INTERACT;
               ready_start = std::chrono::system_clock::now();
            }
            break;
        case L3_State::READY_TO_INTERACT:
            // Wait for a signal or order and move to FOLLOW
            if(std::chrono::duration_cast<chrono::seconds>(std::chrono::system_clock::now()-ready_start).count() > 1)
                l3_state = L3_State::START_FOLLOWING;
            break;
        case L3_State::START_FOLLOWING:
            // check face is no longer visible and body is
            if(l2_people.empty())
            {
                l3_state = L3_State::SEARCHING;
                searching_initial_time = std::chrono::system_clock::now();
                return;
            }
            if( not l2_people.empty()
                and l2_people.front().looking_at == Parts::BODY)
                //and is moving away (velocity is > 0)
            {
                l3_state = L3_State::FOLLOWING;
            }
            break;
        case L3_State::FOLLOWING:
            // distance control
            if(l2_people.empty())
            {
                l3_state = L3_State::START_FOLLOWING;
                return;
            }
            l2_people[0].current_action = Actions::FOLLOW;
            break;
        case L3_State::SEARCHING:
            // person is lost. Wait for L2 and L2 levels to find it
            if(not l2_people.empty())
            {
                l3_state = L3_State::FOLLOWING;
                return;
            }
            if(std::chrono::duration_cast<chrono::seconds>(std::chrono::system_clock::now() - searching_initial_time).count() > 10)
            {
                l3_state = L3_State::WAITING;
            }
            break;
    };
    l3_person.print(l3_state);
}
void SpecificWorker::compute(){};

////////////////////////////////////////////////////////////////////
SpecificWorker::DetectRes SpecificWorker::read_image()
{
    RoboCompCameraSimple::TImage tablet_rgb;
    RoboCompCameraRGBDSimple::TRGBD top_rgbd;
    DetectRes ret{{},{}};
    try
    {
        top_rgbd = camerargbdsimple_proxy->getAll("camera_top");
        tablet_rgb = camerasimple_proxy->getImage();
    }
    catch (const Ice::Exception &e)
    {
        std::cout << e.what() << " Empty image or comms problem" << std::endl;
        return ret;
    }
    const auto &top_image = top_rgbd.image;
    cv::Mat top_rgb_mat = cv::Mat(top_image.height, top_image.width, CV_8UC3, &top_rgbd.image.image[0]);
    const auto &top_depth = top_rgbd.depth;
    cv::Mat top_depth_mat = cv::Mat(top_depth.height, top_depth.width, CV_32F, &top_rgbd.depth.depth[0]);
    const int width = 300;
    const int height = 300;

    if (top_image.width != 0 and top_image.height != 0)
    {
//        cv::Mat img;
//        if (tablet_rgb.compressed)
//        {
//            auto img_uncomp = cv::imdecode(tablet_rgb.image, -1);
//            img = cv::Mat(tablet_rgb.height, tablet_rgb.width, CV_8UC3, &img_uncomp.data);
//        }
//        else
//            img = cv::Mat(tablet_rgb.height, tablet_rgb.width, CV_8UC3, &tablet_rgb.image[0]);

        // from RGBD camera

        cv::Mat n_img;
        cv::resize(top_rgb_mat, n_img, cv::Size(width, height));

        // face
        auto rectangles = face_detector.detect_face_rectangles(n_img);
        if (not rectangles.empty())
        {
            auto r = rectangles.front();
            cv::rectangle(n_img, r, cv::Scalar(0, 105, 205), 3);
            //qInfo() << __FUNCTION__ << "face " << rect;
            float x_scale = (float)(top_depth_mat.cols-1)/n_img.cols;
            float y_scale = (float)(top_depth_mat.rows-1)/n_img.rows;
            QRectF rect = QRectF( x_scale*r.x, y_scale*r.y, x_scale*r.width, y_scale*r.height);
            //qInfo() << __FUNCTION__ << "face " << rect << top_depth_mat.cols << top_depth_mat.rows;
            if(x_scale*r.x + x_scale*r.width > top_depth_mat.cols) rect.setWidth(top_depth_mat.cols-x_scale*r.x);
            if(y_scale*r.y + y_scale*r.height > top_depth_mat.rows) rect.setHeight(top_depth_mat.rows-y_scale*r.y);
            cv::Mat1f roi = top_depth_mat(cv::Rect((int)rect.x(), (int)rect.y(), (int)rect.width(), (int)rect.height()));
            float k = -1;
            if(roi.cols >0 and roi.rows > 0)
            {
                double minVal;
                double maxVal;
                cv::Point minLoc;
                cv::Point maxLoc;
                cv::minMaxLoc(roi, &minVal, &maxVal, &minLoc, &maxLoc);
                k = minVal * 1000; //mm
            }
            //FACE CENTERED in the upper third
            std::get<1>(ret) = {(width/2)-rect.center().x(), (height/3)-rect.center().y(), k};
            //qInfo() << __FUNCTION__ << "Depth face" << k;
        }
        else //body
        {
            // YOLO
            auto people = yolo_detector(n_img);
            if (not people.empty())
            {
                auto r = people.front();
                cv::rectangle(n_img, r, cv::Scalar(0, 0, 255), 3);
                QRectF caca(r.x, r.y, r.width, r.height);
                float x_scale = (float) top_depth_mat.cols / n_img.cols;
                float y_scale = (float) top_depth_mat.rows / n_img.rows;
                QRectF rect = QRectF(x_scale * r.x, y_scale * r.y, x_scale * r.width, y_scale * r.height);
                if (x_scale * r.x + x_scale * r.width > top_depth_mat.cols) rect.setWidth(top_depth_mat.cols - x_scale * r.x - 1);
                if (y_scale * r.y + y_scale * r.height > top_depth_mat.rows) rect.setHeight(top_depth_mat.rows - y_scale * r.y - 1);
                //qInfo() << __FUNCTION__ << "body " << rect << top_depth_mat.cols << top_depth_mat.rows;
                //qInfo() << __FUNCTION__ << "body " << (int)rect.x() << (int)rect.y() << (int)rect.width() << (int)rect.height();
                try
                {
                    cv::Mat roi = top_depth_mat(cv::Rect((int) rect.x(), (int) rect.y(), (int) rect.width(), (int) rect.height()));
                    float k=-1;
                    if(roi.cols > 0 and roi.rows >0)
                    {
                        double minVal;
                        double maxVal;
                        cv::Point minLoc;
                        cv::Point maxLoc;
                        cv::minMaxLoc(roi, &minVal, &maxVal, &minLoc, &maxLoc);
                        k = minVal * 1000; //mm
                    }
                    std::get<0>(ret) = {(width/2) - rect.center().x(), (height/2) - rect.center().y(), k};
                }
                catch(const std::exception &e)
                {
                    std::cout << e.what() << std::endl;
                    qInfo() << __FUNCTION__ << "body " << (int)rect.x() << (int)rect.y() << (int)rect.width() << (int)rect.height();
                };
            }
        }
        // show image
        cv::cvtColor(n_img, n_img, cv::COLOR_BGR2RGB);
        cv::imshow("Camera tablet", n_img);
        cv::waitKey(1);
        return ret;
    }
    else
    {
        qWarning() << "Empty tablet image";
        return ret;
    }
}
void SpecificWorker::move_tablet(std::optional<std::tuple<int,int,int>> body_o, std::optional<std::tuple<int,int,int>> face_o)
{
    float tilt = 0.7;
    float pos = 0;
    if(body_o.has_value())
    {
        auto[_, body_y_error, __] = body_o.value();
        const float delta = 0.1;
        tilt = inverted_tilt * (delta / 100) * body_y_error;  // map from -100,100 to -0.1,0.1 rads
        try
        {
            pos = jointmotorsimple_proxy->getMotorState("tablet_joint").pos;
            //qInfo() << __FUNCTION__ << "BODY: pos" << pos << "error" << body_y_error << "tilt" << tilt << pos - tilt;
            jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{pos - tilt, 1});
        }  // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to join motor " << std::endl; }
    }
    else if( face_o.has_value())
    {
        auto[_, face_y_error, __] = face_o.value();
        const float delta = 0.1;
        tilt = inverted_tilt * (delta / 100) * face_y_error;  // map from -100,100 to -0.1,0.1 rads
        try
        {
            pos = jointmotorsimple_proxy->getMotorState("tablet_joint").pos;
            // qInfo() << __FUNCTION__ << "FACE: pos" << pos << "error" << face_y_error << "tilt" << tilt << pos - tilt;
            if(fabs(pos-tilt) > 0.17)
                jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{pos - tilt, 1});
        } // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to join motor" << std::endl; }
    }
//    if (abs(tilt) > 0.1 )
//    {
//        try { jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{pos - tilt, 1}); }  // radians. 0 vertical
//        catch (const Ice::Exception &e) { std::cout << e.what() <<  " No connection to join motor" << std::endl; }
//    }
}
void SpecificWorker::move_base(std::optional<std::tuple<int,int,int>> body_o,
                               std::optional<std::tuple<int,int,int>> face_o)
{
    // rotate base
    float advance = 0.0;
    float rot = 0.0;
    const float MAX_ADVANCE_SPEED = 500;
    const float MIN_PERSON_DISTANCE = 700;
    if(body_o.has_value())
    {
        auto &[body_x_error, _, body_dist] = body_o.value();
        rot = -(2.f / 100) * body_x_error;
        if(l1_person.current_action == Actions::FOLLOW)
        {
            if(fabs(body_dist-MIN_PERSON_DISTANCE) < 100)
                advance = 0;
            if((body_dist-MIN_PERSON_DISTANCE) < -100)
                advance = std::clamp(body_dist-MIN_PERSON_DISTANCE, -300.f, 0.f);
            else
                advance = MAX_ADVANCE_SPEED * (body_dist-MIN_PERSON_DISTANCE)*(1/1000.f) * exp(-rot*rot*2);
        }
//        if(body_dist > 0 and body_dist < 400)
//            advance = -(400.0 / 400.0) * body_dist;

    }
    else if(face_o.has_value())
    {
        auto &[face_x_error, _, face_dist] = face_o.value();
        rot = -(2.f / 100) * face_x_error;
//        if(face_dist > 0 and face_dist < 400)
//            advance = -(400.0 / 400.0) * face_dist;
    }

    //if (abs(rot) > 0.5 )  // avoid sending small velocities
    {
        try
        {
            const float gain = 0.4;
            differentialrobot_proxy->setSpeedBase(advance, gain * rot);
            robot.current_rot_speed = gain*rot;
            robot.current_adv_speed = advance;
        }
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
        //qInfo() << __FUNCTION__ << "FACE: pos" << pos << "error" << face_y_error << "tilt" << tilt << pos - tilt;
        try { emotionalmotor_proxy->pupposition(tilt_x,tilt_y); }  // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << " No connection to emotional motor" << std::endl; }
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