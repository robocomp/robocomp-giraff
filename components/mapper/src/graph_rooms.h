//
// Created by pbustos on 8/12/21.
//

#ifndef MAPPER_GRAPH_ROOMS_H
#define MAPPER_GRAPH_ROOMS_H

// thanks to https://github.com/CheckBoxStudio/IoU

#include "iou.h"
#include <Eigen/Geometry>
#include <QtCore>
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>
#include <set>
#include <vector>

class Graph_Rooms
{
    public:
        struct Room
        {
           public:
                IOU::Quad quad;
                int id;
                IOU::Vertexes points;
                const float diff = 300;
                QPointF graph_pos;
                bool operator == (const Room &d)
                {
                    double iou = IOU::iou(quad, d.quad);
                    return iou > 0.9;
                }
                Room(const IOU::Quad &quad_, int id_) : quad(quad_), id(id_)
                {
                    quad.beInClockWise();
                    quad.getVertList(points);
                };
                void print()
                {
                    qInfo() << "Room:" << id;
                    qInfo() << "    graph_pos:" << graph_pos;
                }
                void draw(QGraphicsScene *scene)
                {
                    QPolygonF poly;
                    for (auto p: points)
                        poly << QPointF(p.x, p.y);
                    auto v = scene->addPolygon(poly, QPen(QColor("Blue"), 90));
                    v->setZValue(100);
                }
        };
        struct Door
        {
            public:
                Eigen::Vector2f p1,p2;
                int id;
                std::set<int> to_rooms;
                const float diff = 400;
                float width() const {return (p1-p2).norm();}
                bool operator ==(const Door &d) { return ((d.p1-p1).norm() < diff and (d.p2-p2).norm() < diff) or
                                                         ((d.p1-p2).norm() < diff and (d.p2-p1).norm() < diff);};
                Eigen::Vector2f get_midpoint() const {return p1 + ((p2-p1)/2.0);};
                Eigen::Vector2f get_external_midpoint(int room, const Eigen::Vector2f &robot) const
                {
                    Eigen::ParametrizedLine<float, 2> r1 =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p1-p2).unitOrthogonal());
                    Eigen::ParametrizedLine<float, 2> r2 =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p2-p1).unitOrthogonal());
                    if ((robot-r1.pointAt(1300)).norm() > (robot-r2.pointAt(1300)).norm()) return r1.pointAt(1300);
                    else return r2.pointAt(1300);
                };
                bool connects_to_room(int room) const { return to_rooms.contains(room);};
                void operator=(const Door &d){ p1 = d.p1; p2=d.p2; to_rooms=d.to_rooms;};
                float distance_to_robot(const Eigen::Vector2f &robot) const { return (get_midpoint() - robot).norm(); };
                void print()
                {
                    qInfo() << "Door:" << id;
                    qInfo() << "    p1:" << p1.x() << p1.y();
                    qInfo() << "    p2:" << p2.x() << p2.y();
                    qInfo() << "    num rooms:" << to_rooms.size();
                    for (const auto &r: to_rooms)
                        qInfo() << "    to room -> " << r;
                }
        };

        std::vector<Door> doors;
        std::vector<Room> rooms;

        void draw_node(int id, QGraphicsScene *scene);
        void draw_doors(const std::vector<Door> &local_doors, QGraphicsScene *scene);
        void draw_edge(int door_id, QGraphicsScene *scene);
        void flip_text(QGraphicsTextItem *text);
};


#endif //MAPPER_GRAPH_ROOMS_H
