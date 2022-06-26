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
#include <gsl_assert/assert>
#include <uuid/uuid.h>

class Graph_Rooms
{
    public:
        Graph_Rooms();
        struct Door
        {
                Door(const Eigen::Vector2f &p1_, const Eigen::Vector2f &p2_, const std::string &id_) : p1_in_grid(p1_), p2_in_grid(p2_), id(id_)
                {};
                Eigen::Vector2f p1_in_grid,p2_in_grid;   // line coordinates in grid RS
                Eigen::Vector2f p1_in_rect, p2_in_rect;  // line coordinates in rect RS
                Eigen::Vector2f p1_in_world, p2_in_world;  // line coordinates in world RS to draw
                std::string id;
                struct From_Room
                {
                    int room_id = -1;
                    Eigen::Vector2f p1,p2;   // line coordinates in this room RS
                };
                std::map<int, From_Room> my_rooms;
                const float diff = 400;
                QGraphicsItem *poly_draw = nullptr;
                float width() const {return (p1_in_grid-p2_in_grid).norm();}
                bool operator ==(const Door &d) { return ((d.p1_in_grid-p1_in_grid).norm() < diff and (d.p2_in_grid-p2_in_grid).norm() < diff) or
                                                         ((d.p1_in_grid-p2_in_grid).norm() < diff and (d.p2_in_grid-p1_in_grid).norm() < diff);};
                float distance_to(const Door &d) { return std::min((d.p1_in_grid-p1_in_grid).norm() + (d.p2_in_grid-p2_in_grid).norm(),
                                                                   (d.p1_in_grid-p2_in_grid).norm() + (d.p2_in_grid-p1_in_grid).norm());};
                Eigen::Vector2f get_midpoint() const {return p1_in_grid + ((p2_in_grid-p1_in_grid)/2.0);};
                Eigen::Vector2f get_external_midpoint(const Eigen::Vector2f &inside_point, int dist = 1500) const
                {
                    Eigen::ParametrizedLine<float, 2> r1 =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p1_in_grid-p2_in_grid).unitOrthogonal());
                    Eigen::ParametrizedLine<float, 2> r2 =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p2_in_grid-p1_in_grid).unitOrthogonal());
                    if ((inside_point-r1.pointAt(dist)).norm() > (inside_point-r2.pointAt(dist)).norm())
                        return r1.pointAt(dist);
                    else
                        return r2.pointAt(dist);
                };
                Eigen::Vector2f get_midpoint_world() const {return p1_in_world + ((p2_in_world-p1_in_world)/2.0);};
                Eigen::Vector2f get_external_midpoint_world(const Eigen::Vector2f &inside_point, int dist = 1500) const
                {
                    Eigen::ParametrizedLine<float, 2> r1 =  Eigen::ParametrizedLine<float, 2>(get_midpoint_world(), (p1_in_world-p2_in_world).unitOrthogonal());
                    Eigen::ParametrizedLine<float, 2> r2 =  Eigen::ParametrizedLine<float, 2>(get_midpoint_world(), (p2_in_world-p1_in_world).unitOrthogonal());
                    if ((inside_point-r1.pointAt(dist)).norm() > (inside_point-r2.pointAt(dist)).norm())
                        return r1.pointAt(dist);
                    else
                        return r2.pointAt(dist);
                };
                Door& operator=(Door other)
                {
                    std::swap(id, other.id);
                    std::swap(p1_in_grid, other.p1_in_grid);
                    std::swap(p2_in_grid, other.p2_in_grid);
                    std::swap(p1_in_world, other.p1_in_world);
                    std::swap(p2_in_world, other.p2_in_world);
                    std::swap(p1_in_rect, other.p1_in_rect);
                    std::swap(p2_in_rect, other.p2_in_rect);
                    poly_draw = other.poly_draw;
                    return *this;
                }
                std::optional<int> get_the_other_room(int room_id) const
                {
                    for(const auto &[k, v] : my_rooms)
                        if(k != room_id)
                            return k;
                    return {};
                }
                bool has_only_one_room() const { return my_rooms.size() == 1; };
                std::optional<int> get_the_only_room() const
                {
                    if (my_rooms.size() == 1)
                        return my_rooms.begin()->first;
                    else
                        return {};
                };
                float distance_to_robot(const Eigen::Vector2f &robot) const { return (get_midpoint() - robot).norm(); };
                void print()
                {
                    qInfo() << "    Door:" << QString::fromStdString(id);
                    qInfo() << "     p1_grid:" << p1_in_grid.x() << p1_in_grid.y();
                    qInfo() << "     p2_grid:" << p2_in_grid.x() << p2_in_grid.y();
                    qInfo() << "     p1_world:" << p1_in_world.x() << p1_in_world.y();
                    qInfo() << "     p2_world:" << p2_in_world.x() << p2_in_world.y();
                    qInfo() << "     p1_rect:" << p1_in_rect.x() << p1_in_rect.y();
                    qInfo() << "     p2_rect:" << p2_in_rect.x() << p2_in_rect.y();
                    for(auto &[k, r] : my_rooms)
                        qInfo() << "     connects to room:" << r.room_id;

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

                    poly_draw = scene->addLine(p1_in_world.x(), p1_in_world.y(), p2_in_world.x(), p2_in_world.y(), QPen(QColor("Magenta"), 50));
                    poly_draw->setZValue(10);
                }
            };
        struct Room
        {
            Room(int id_) : id(id_)
            {

            };
            Room(Room const&) = default;
            Room& operator=(Room other)
            {
                std::swap(quad, other.quad);
                std::swap(id, other.id);
                std::swap(points, other.points);
                std::swap(room_world_rect, other.room_world_rect);
                std::swap(room_rect, other.room_rect);
                std::swap(doors_ids, other.doors_ids);
                is_unknown = other.is_unknown;
                poly_draw = other.poly_draw;
                return *this;
            }
            IOU::Quad quad;   //eliminate
            int id;
            cv::RotatedRect room_rect, room_world_rect;
            IOU::Vertexes points;
            QGraphicsItem *poly_draw = nullptr;
            std::vector<std::string> doors_ids;
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
            void print()
            {
                qInfo() << "Room:" << id;
                qInfo() << "    is_unknown:" << is_unknown;
                qInfo() << "    rect:" << room_rect.center.x << room_rect.center.y << room_rect.size.width << room_rect.size.height;
                qInfo() << "    rect_world:" << room_world_rect.center.x << room_world_rect.center.y << room_world_rect.size.width << room_world_rect.size.height;
            }
            void draw(QGraphicsScene *scene)
            {
                //if(poly_draw != nullptr)
                //    scene->removeItem(poly_draw);

                cv::Point2f rect_points[4];
                room_world_rect.points(rect_points);
                QPolygonF pol;
                for(const auto &p: rect_points)
                    pol << QPointF(p.x, p.y);
                poly_draw = scene->addPolygon(pol, QPen(QColor("Blue"), 50));
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

        std::map<int, Room> rooms;   // so they can be used inside Door and Room
        std::map<std::string, Door> doors;

        Room& current_room() { return rooms.at(current_room_local);};
        Room current_room() const { return rooms.at(current_room_local);};
        void set_current_room(size_t id) { current_room_local = id;};
        Door& current_door() { return doors.at(current_door_local);};
        Door current_door() const { return doors.at( current_door_local);};
        void set_current_door(const std::string &id) { current_door_local = id;};
        bool there_is_a_current_door() const { return current_door_local != "none";};
        void merge_doors(int room_orig, const std::string &door_orig, int room_dest, const std::string &door_dest);
        void change_current_room_key_to(int key)
        {
            auto nodeHandler = rooms.extract(current_room_local);
            nodeHandler.key() = key;
            rooms.insert(std::move(nodeHandler));
            current_room_local = key;
        }
        void draw_nodes(QGraphicsScene *scene);
        void draw_doors(QGraphicsScene *scene);
        void draw_rooms(QGraphicsScene *scene);
        void draw_edges(QGraphicsScene *scene);
        void draw_all(QGraphicsScene *robot_scene, QGraphicsScene *graph_scene);
        void flip_text(QGraphicsTextItem *text);
        void add_door_to_current_room(const Eigen::Vector2f &p1_grid, const Eigen::Vector2f &p2_grid);
        Eigen::Matrix<double, 4, 3> get_room_sides_matrix(const Room &r);
        Eigen::Vector2f  project_point_on_closest_side(const Room &room, const Eigen::Vector2f &p);
        float  min_distance_from_point_to_closest_side(const Room &r, const Eigen::Vector2f &p) const;
        void project_doors_on_room_side(Room &r, QGraphicsScene *scene);
        std::optional<std::string> get_door_with_only_one_room(int room) const;
        void remove_door(const std::string &id);
        void remove_doors_not_parallel_walls(Room &r);
        void print();
    private:
        int current_room_local = -1;
        std::string current_door_local = "none";
};


#endif //MAPPER_GRAPH_ROOMS_H