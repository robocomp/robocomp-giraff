//
// Created by pbustos on 5/4/21.
//

#ifndef CONTROLLER_DSR_PLAN_H
#define CONTROLLER_DSR_PLAN_H

class Plan
{
    public:
        Plan()
        {
            empty = true;
            active = false;
        };

        Plan(const std::string &plan_string)
        {
            QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(plan_string).toUtf8());
            planJ = qvariant_cast<QVariantMap>(doc.toVariant());
            QString act = planJ.keys().front();  // OJO ES SOLO LA PRIMERA KEY DEL MAPA
            this->action = strings_to_actions.at(act.toStdString());
        };

        void reset()
        {
            planJ.clear();
            empty = true;
            action = Plan::Actions::NONE;
            std::cout << __FUNCTION__ << pprint() << std::endl;
        }
        std::string to_json() const
        {
            QJsonDocument json = QJsonDocument::fromVariant(planJ);
            //QTextStream ts(stdout);
            //ts << json.toJson();
            return QString(json.toJson()).toStdString();
        }
        std::string pprint() const
        {
            std::stringstream ss;
            ss << "Action: " << actions_to_strings.at(action) << std::endl;
            for(const auto p : planJ.keys())
                for(const auto e : qvariant_cast<QVariantMap>(planJ[p]).keys())
                {
                    ss << "\t" << e.toStdString() << " : " << qvariant_cast<QVariantMap>(planJ[p])[e].toString().toStdString() << std::endl;
                }
           return ss.str();
        };
        //Eigen::Vector3d get_target_trans() const { return Eigen::Vector3d(params.at("x"), params.at("y"), 0.f);};
        void set_active(bool s) {active = s;};
        bool is_active() const {return active;};
        bool is_empty() const { return empty; };
        void set_empty(bool e) { empty = e;};
        QPointF get_target() const
        {
            float x = qvariant_cast<QVariantMap>(planJ["GOTO"]).value("x").toFloat();
            float y = qvariant_cast<QVariantMap>(planJ["GOTO"]).value("y").toFloat();
            return QPointF(x, y);
        }

        enum class Actions {NONE, GOTO, BOUNCE, FOLLOW_PATH};
        Actions action;
        bool is_action(Actions test) const
        {
            return test == this->action;
        };

        bool is_complete()
        {
            return action_to_tests.at(action);
        }

        //qmap
        QVariantMap planJ;

    private:
        bool empty = true;
        bool active = false;
        std::map<Actions, std::string> actions_to_strings
         {
            {Actions::GOTO, "GOTO"},
            {Actions::BOUNCE, "BOUNCE"},
            {Actions::FOLLOW_PATH, "FOLLOW_PATH"},
            {Actions::NONE, "NONE"}
         };
        std::map<std::string, Actions> strings_to_actions
        {
            {"GOTO", Actions::GOTO},
            {"BOUNCE", Actions::BOUNCE},
            {"FOLLOW_PATH", Actions::FOLLOW_PATH},
            {"NONE", Actions::NONE}
        };
        // Tests for specific plans
        typedef bool (Plan::*Test)();
        bool GOTO_test()
        {
            auto params = qvariant_cast<QVariantMap>(planJ["GOTO"]);
            if(not params.contains("x"))
            { qWarning() << __FUNCTION__ << "Missing x"; return false; }
            if(not params.contains("y"))
            { qWarning() << __FUNCTION__ << "Missing y"; return false; }
//            if(not params.contains("angle"))
//            { qWarning() << __FUNCTION__ << "Missing angle"; return false; }
            return true;
        };
        bool BOUNCE_test() { return true; };
        bool FOLLOW_PATH_test() { return true; };
        std::map <Actions, Test> action_to_tests
        {
            {Actions::GOTO, &Plan::GOTO_test},
            {Actions::BOUNCE, &Plan::BOUNCE_test},
            {Actions::FOLLOW_PATH, &Plan::FOLLOW_PATH_test}
        };
};

#endif //CONTROLLER_DSR_MISSION_H


//        Plan(const std::string &plan_string_)
//        {
//            QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(plan_string_).toUtf8());
//            QJsonObject planJson = doc.object();
//            QJsonArray actionArray = planJson.value("plan").toArray();
//            QJsonObject action_0 = actionArray.at(0).toObject();
//            QString action_s = action_0.value("action").toString();
//            if (action_s == "GOTO")
//            {
//                QJsonObject action_params = action_0.value("params").toObject();
//                QString object = action_params.value("object").toString();
//                QJsonArray location = action_params.value("location").toArray();
//                params["x"] = location.at(0).toDouble();
//                params["y"] = location.at(1).toDouble();
//                params["angle"] = location.at(2).toDouble();
//                action = Plan::Actions::GOTO;
//                target_place = object.toStdString();
//            }
//            if (action_s == "BOUNCE")
//                action = Plan::Actions::BOUNCE;
//            if (action_s == "FOLLOW_PATH")
//                action = Plan::Actions::FOLLOW_PATH;
//
//            plan_string = plan_string_;
//            empty = false;
//        };
