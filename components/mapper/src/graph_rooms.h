//
// Created by pbustos on 8/12/21.
//

#ifndef MAPPER_GRAPH_ROOMS_H
#define MAPPER_GRAPH_ROOMS_H

// thanks to https://github.com/CheckBoxStudio/IoU

#include "iou.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <QtCore>
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>
#include <set>
#include <vector>
#include <cppitertools/range.hpp>
#include <iostream>
#include <opencv2/core/types.hpp>

class Graph_Rooms
{
    public:
        Graph_Rooms();
        struct Door
        {
            public:
                Eigen::Vector2f p1,p2;   // line coordinates
                int id;
                struct From_Room
                {
                    int room_id = -1;
                    Eigen::Vector2f p1,p2;   // line coordinates in this room RS
                };
                std::map<int, From_Room> rooms;

                int to_room = -1;
                int from_room = -1;
                const float diff = 400;
                QGraphicsItem *poly_draw = nullptr;
                float width() const {return (p1-p2).norm();}
                bool operator ==(const Door &d) { return ((d.p1-p1).norm() < diff and (d.p2-p2).norm() < diff) or
                                                         ((d.p1-p2).norm() < diff and (d.p2-p1).norm() < diff);};
                Eigen::Vector2f get_midpoint() const {return p1 + ((p2-p1)/2.0);};
                Eigen::Vector2f get_external_midpoint(const Eigen::Vector2f &inside_point, int dist = 1200) const
                {
                    Eigen::ParametrizedLine<float, 2> r1 =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p1-p2).unitOrthogonal());
                    Eigen::ParametrizedLine<float, 2> r2 =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p2-p1).unitOrthogonal());
                    if ((inside_point-r1.pointAt(dist)).norm() > (inside_point-r2.pointAt(dist)).norm())
                        return r1.pointAt(dist);
                    else
                        return r2.pointAt(dist);
                };
                Door& operator=(Door other)
                {
                    std::swap(id, other.id);
                    std::swap(p1, other.p1);
                    std::swap(p2, other.p2);
                    std::swap(to_room, other.to_room);
                    std::swap(from_room, other.from_room);
                    poly_draw = other.poly_draw;
                    return *this;
                }
                float distance_to_robot(const Eigen::Vector2f &robot) const { return (get_midpoint() - robot).norm(); };
                void print()
                {
                    qInfo() << "    Door:" << id;
                    qInfo() << "     p1:" << p1.x() << p1.y();
                    qInfo() << "     p2:" << p2.x() << p2.y();
                    qInfo() << "     to_room:" << to_room;
                    qInfo() << "     from_room:" << from_room;

//                    qInfo() << "    num rooms:" << to_rooms.size();
//                    for (const auto &r: to_rooms)
//                        qInfo() << "    to room -> " << r;
                }
               void draw(QGraphicsScene *scene, const Eigen::Vector2f &pos = Eigen::Vector2f(0.f,0.f), float ang=0.f)
                {
                    if(poly_draw != nullptr)
                        scene->removeItem(poly_draw);

                    // rect to grid
//                    Eigen::Matrix2f matrix;
//                    matrix << cos(ang), -sin(ang), sin(ang) ,cos(ang);
//                    auto pp1 = matrix * rooms.at(0).p1 + Eigen::Vector2f(room_center.x, room_Center.y);
//                    auto pp2 = matrix * rooms.at(0).p2 + Eigen::Vector2f(room_center.x, room_Center.y);

                    // grid to world
                    Eigen::Matrix2f g2w;
                    g2w <<  cos(ang), -sin(ang),
                            sin(ang), cos(ang);
                    auto pp1 = g2w * p1 + pos;
                    auto pp2 = g2w * p2 + pos;
                    //std::cout << __FUNCTION__ << "p1 " << p1 << " p2 " << p2 << std::endl;
                    //std::cout << __FUNCTION__ << "mat " << g2w << "pp1 " << pp1 << " pp2 " << pp2 << std::endl;
                    poly_draw = scene->addLine(pp1.x(), pp1.y(), pp2.x(), pp2.y(), QPen(QColor("Magenta"), 50));
                    poly_draw->setZValue(10);
                }
        };
        struct Room
        {
           public:
                IOU::Quad quad;   //eliminate
                int id;
                cv::RotatedRect room_rect;
                IOU::Vertexes points;
                QPointF graph_pos;
                QGraphicsItem *poly_draw = nullptr;
                std::vector<int> doors_ids;
                bool is_unknown = true;
                double get_witdh() const { return room_rect.size.width;};
                double get_height() const { return room_rect.size.height;};
                double get_semi_witdh() const { return room_rect.size.width/2.0;};
                double get_semi_height() const { return room_rect.size.height/2.0;};
                bool operator == (const Room &d)
                {
                    double iou = IOU::iou(quad, d.quad);
                    return iou > 0.9;
                }
                Room(int id_) : id(id_) {};
//                Room(const IOU::Quad &quad_, int id_, const QRectF &rect) : quad(quad_), id(id_), room_rect(rect)
//                {
//                    quad.beInClockWise();
//                    quad.getVertList(points);
//                };
                Room(Room const&) = default;
                Room& operator=(Room other)
                {
                    std::swap(quad, other.quad);
                    std::swap(id, other.id);
                    std::swap(points, other.points);
                    std::swap(graph_pos, other.graph_pos);
                    std::swap(room_rect, other.room_rect);
                    std::swap(doors_ids, other.doors_ids);
                    is_unknown = other.is_unknown;
                    poly_draw = other.poly_draw;
                    return *this;
                }
                void print()
                {
                    qInfo() << "Room:" << id;
                    qInfo() << "    graph_pos:" << graph_pos;
                    qInfo() << "    rect:" << room_rect.center.x << room_rect.center.y << room_rect.size.width << room_rect.size.height;
//                    for(auto &d: doors_ids)
//                        doors[id].print();
                }
                void draw(QGraphicsScene *scene, const Eigen::Vector2f &offset=Eigen::Vector2f(0.f,0.f), float ang=0.f)
                {
                    //if(poly_draw != nullptr)
                    //    scene->removeItem(poly_draw);

                    auto r = room_rect;
                    r.center = r.center + cv::Point2f(offset.x(), offset.y());
                    r.angle += qRadiansToDegrees(ang);
                    //poly_draw = scene->addRect(r, QPen(QColor("Blue"), 90));
                    cv::Point2f rect_points[4];
                    r.points(rect_points);
                    QPolygonF pol;
                    for(const auto &p: rect_points)
                        pol << QPointF(p.x, p.y);
                    poly_draw = scene->addPolygon(pol, QPen(QColor("Blue"), 90));
                    graph_pos = QPointF(r.center.x, r.center.y);
                    poly_draw->setZValue(10);
                }
                void add_step_to_width(double step) { room_rect.size.width = room_rect.size.width + step;};
                void add_step_to_height(double step) { room_rect.size.height = room_rect.size.height + step;};
                void add_step_to_center_x(double step) { room_rect.center = room_rect.center + cv::Point2f{(float)step, 0.f};};
                void add_step_to_center_y(double step) { room_rect.center = room_rect.center + cv::Point2f{0.f, (float)step};};
            //void add_step_to_width(double step) { room_rect.setWidth(room_rect.width() + step);};
            //                void add_step_to_height(double step) {room_rect.setHeight(room_rect.height() + step);};
            //                void add_step_to_center_x(double step) { room_rect.moveCenter(room_rect.center() + QPointF(step, 0.f));};
            //                void add_step_to_center_y(double step) {room_rect.moveCenter(room_rect.center() + QPointF(0.f, step));};
        };

        std::vector<Room> rooms;
        std::vector<Door> doors;
        int current_room_local = 0;
        bool is_current_room_unknown = true;
        int number_of_doors = 0;

        Room& current_room() { return rooms.at(current_room_local);};
        Room current_room() const { return rooms.at(current_room_local);};
        void draw_nodes(QGraphicsScene *scene);
        void draw_doors(QGraphicsScene *scene);
        void draw_rooms(QGraphicsScene *scene);
        void draw_edges(QGraphicsScene *scene);
        void draw_all(QGraphicsScene *robot_scene, QGraphicsScene *graph_scene);
        void flip_text(QGraphicsTextItem *text);
        void add_door_to_current_room(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2);
        Eigen::Matrix<double, 4, 3> get_room_sides_matrix(const Room &r);
        Eigen::Vector2f  project_point_on_closest_side(const Room &room, const Eigen::Vector2f &p);
        float  min_distance_from_point_to_closest_side(const Room &r, const Eigen::Vector2f &p) const;
        void project_doors_on_room_side(Room &r, QGraphicsScene *scene);
};


#endif //MAPPER_GRAPH_ROOMS_H
