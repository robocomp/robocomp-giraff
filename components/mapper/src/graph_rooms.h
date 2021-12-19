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

class Graph_Rooms
{
    public:
        Graph_Rooms();
        struct Door
        {
            public:
                Eigen::Vector2f p1,p2;
                int id;
                int to_room = -1;
                int from_room = -1;
                const float diff = 400;
                QGraphicsItem *poly_draw = nullptr;
                float width() const {return (p1-p2).norm();}
                bool operator ==(const Door &d) { return ((d.p1-p1).norm() < diff and (d.p2-p2).norm() < diff) or
                                                         ((d.p1-p2).norm() < diff and (d.p2-p1).norm() < diff);};
                Eigen::Vector2f get_midpoint() const {return p1 + ((p2-p1)/2.0);};
                Eigen::Vector2f get_external_midpoint(const Eigen::Vector2f &inside_point) const
                {
                    Eigen::ParametrizedLine<float, 2> r1 =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p1-p2).unitOrthogonal());
                    Eigen::ParametrizedLine<float, 2> r2 =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p2-p1).unitOrthogonal());
                    if ((inside_point-r1.pointAt(1300)).norm() > (inside_point-r2.pointAt(1300)).norm())
                        return r1.pointAt(1300);
                    else
                        return r2.pointAt(1300);
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
                //bool connects_to_room(int room) const { return to_rooms.contains(room);};
                //void operator=(const Door &d){ p1 = d.p1; p2=d.p2; to_rooms=d.to_rooms;};
                float distance_to_robot(const Eigen::Vector2f &robot) const { return (get_midpoint() - robot).norm(); };
                void print()
                {
                    qInfo() << "Door:" << id;
                    qInfo() << "    p1:" << p1.x() << p1.y();
                    qInfo() << "    p2:" << p2.x() << p2.y();
                    qInfo() << "    to_room:" << to_room;
                    qInfo() << "    from_room:" << from_room;

//                    qInfo() << "    num rooms:" << to_rooms.size();
//                    for (const auto &r: to_rooms)
//                        qInfo() << "    to room -> " << r;
                }
                void draw(QGraphicsScene *scene)
                {
                    if(poly_draw != nullptr)
                        scene->removeItem(poly_draw);
                    poly_draw = scene->addLine(p1.x(), p1.y(), p2.x(), p2.y(), QPen(QColor("Magenta"), 50));
                    poly_draw->setZValue(200);
                }
        };
        struct Room
        {
           public:
                IOU::Quad quad;   //eliminate
                int id;
                QRectF room_rect;
                IOU::Vertexes points;
                QPointF graph_pos;
                QGraphicsItem *poly_draw = nullptr;
                std::vector<Door> doors;
                bool is_unknown = true;
                double get_witdh() const { return room_rect.width();};
                double get_height() const { return room_rect.height();};
                double get_semi_witdh() const { return room_rect.width()/2.0;};
                double get_semi_height() const { return room_rect.height()/2.0;};
                bool operator == (const Room &d)
                {
                    double iou = IOU::iou(quad, d.quad);
                    return iou > 0.9;
                }
                Room(int id_) : id(id_) {};
                Room(const IOU::Quad &quad_, int id_, const QRectF &rect) : quad(quad_), id(id_), room_rect(rect)
                {
                    quad.beInClockWise();
                    quad.getVertList(points);
                };
                Room(Room const&) = default;
                Room& operator=(Room other)
                {
                    std::swap(quad, other.quad);
                    std::swap(id, other.id);
                    std::swap(points, other.points);
                    std::swap(graph_pos, other.graph_pos);
                    std::swap(room_rect, other.room_rect);
                    std::swap(doors, other.doors);
                    is_unknown = other.is_unknown;
                    poly_draw = other.poly_draw;
                    return *this;
                }
                void print()
                {
                    qInfo() << "Room:" << id;
                    qInfo() << "    graph_pos:" << graph_pos;
                    for(auto &d: doors)
                        d.print();
                }
                void draw(QGraphicsScene *scene)
                {
                    if(poly_draw != nullptr)
                        scene->removeItem(poly_draw);
                    poly_draw = scene->addRect(room_rect, QPen(QColor("Blue"), 90));
                    graph_pos = room_rect.center();
                    poly_draw->setZValue(100);
                }
                double distance_to_door(const Door &d)  const   // compute the shortest distance of door d to this room
                {
                    // compute distance to all four sides
                    Eigen::Vector2d c1(room_rect.right(), room_rect.top());
                    Eigen::Vector2d c2(room_rect.right(), room_rect.bottom());
                    Eigen::Vector2d c3(room_rect.left(), room_rect.bottom());
                    Eigen::Vector2d c4(room_rect.left(), room_rect.top());
                    Eigen::Matrix<double, 4, 3> sides_matrix;
                    sides_matrix << c1.y() - c2.y(), c2.x() - c1.x(), (c1.x() - c2.x()) * c1.y() + (c2.y() - c1.y()) * c1.x(),
                            c2.y() - c3.y(), c3.x() - c2.x(), (c2.x() - c3.x()) * c2.y() + (c3.y() - c2.y()) * c2.x(),
                            c3.y() - c4.y(), c4.x() - c3.x(), (c3.x() - c4.x()) * c3.y() + (c4.y() - c3.y()) * c3.x(),
                            c4.y() - c1.y(), c1.x() - c4.x(), (c4.x() - c1.x()) * c4.y() + (c1.y() - c4.y()) * c4.x();
                    // compute normalizing line coefficients M 4x1
                    Eigen::Vector4d M;
                    for( auto &&i : iter::range(sides_matrix.rows()))
                        M[i] = sqrt(sides_matrix.row(i).x()*sides_matrix.row(i).x() + sides_matrix.row(i).y()*sides_matrix.row(i).y());
                    // multiply door points times line coefficients: dist = 4x3 * 3*1 = 4x1
                    auto dmp = d.get_midpoint();
                    Eigen::Vector3d door_mid_point(dmp.x() ,dmp.y(), 1.0);
                    auto dist = sides_matrix * door_mid_point;  // 4x1
                    // compute absolute value
                    auto abs_dist = dist.cwiseAbs();
                    // divide by norm coeffs  4x1 / colwise
                    auto abs_dist_norm = abs_dist.array().colwise() / M.array();
                    // minimun value along columns
                    double res = abs_dist_norm.colwise().minCoeff().x();
                    //qInfo() << __FUNCTION__ << "RES SIZE:" << abs_dist.colwise().minCoeff().rows() <<  abs_dist.colwise().minCoeff().cols() << res;
                    return res;
                }
                void add_step_to_width(double step) { room_rect.setWidth(room_rect.width() + step);};
                void add_step_to_height(double step) {room_rect.setHeight(room_rect.height() + step);};
        };

        //std::vector<Door> doors;
        std::vector<Room> rooms;
        int current_room_local = 0;
        bool is_current_room_unknown = true;
        int number_of_doors = 0;

        Room& current_room() { return rooms.at(current_room_local);};
        Room current_room() const { return rooms.at(current_room_local);};
        void draw_node(const Room &r, QGraphicsScene *scene);
        void draw_doors(QGraphicsScene *scene);
        void draw_rooms(QGraphicsScene *scene);
        void draw_edges(QGraphicsScene *scene);
        void flip_text(QGraphicsTextItem *text);
        void add_door_to_current_room(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2);
        Eigen::Matrix<double, 4, 3> get_room_sides_matrix(const Room &r);
        Eigen::Vector2f  project_point_on_closest_side(const Room &room, const Eigen::Vector2f &p);
};


#endif //MAPPER_GRAPH_ROOMS_H
