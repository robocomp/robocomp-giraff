//
// Created by pbustos on 8/12/21.
//

#include "graph_rooms.h"
#include <cppitertools/filter.hpp>

Graph_Rooms::Graph_Rooms()
{
    //rooms.insert(std::make_pair(-1, Room(-1)));  // initial unknown room on creation. Replace when graph is loaded from disk
    //current_room_local = -1;
};

void Graph_Rooms::draw_doors(QGraphicsScene *scene)
{
    std::vector<QGraphicsItem *> nodes;
    for(const auto &e : nodes)
        scene->removeItem(e);
    nodes.clear();

    for (auto &[k, r] : rooms)
        for(auto &d : r.doors_ids)
            doors.at(d).draw(scene);
}
void Graph_Rooms::draw_rooms(QGraphicsScene *scene)
{
    for (auto &[kk, rr]: rooms)
        rr.draw(scene);
}
void Graph_Rooms::draw_nodes(QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem*> nodes;
    for(auto n: nodes)
    {
        for (auto c: n->childItems())
            scene->removeItem(c);
        scene->removeItem(n);
    }

    for(const auto &[k, r] : rooms)
    {
        QGraphicsEllipseItem *node;
        if(k != current_room().id)
            node = scene->addEllipse(0, 0, 600, 600, QPen(QColor("lightBlue"), 50), QBrush(QColor("lightBlue")));
        else
            node = scene->addEllipse(0, 0, 600, 600, QPen(QColor("orange"), 50), QBrush(QColor("orange")));

        auto x = r.room_world_rect.center.x;
        auto y = r.room_world_rect.center.y;
        node->setPos(x - 300, y - 300);
        node->setZValue(100);
        QFont f;
        f.setPointSize(180);
        auto text = scene->addText("r-" + QString::number(r.id), f);
        text->setParentItem(node);
        flip_text(text);
        text->setPos(180, 400);
        node->setZValue(120);
        nodes.push_back(node);
    }
}
void Graph_Rooms::draw_edges(QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem *> gdoors;
    static std::vector<QGraphicsItem *> texts;
    static std::vector<QGraphicsLineItem *> edges;
    for(auto &t: texts)
        scene->removeItem(t);
    for(auto &e: edges)
        scene->removeItem(e);
    for(auto &d: gdoors)
        scene->removeItem(d);
    gdoors.clear();
    texts.clear();
    edges.clear();

    std::set<int> doors_in_rooms;  // to avoid drawing two one-room doors in the same spot
    for(const auto &[kd, door]: doors)
    {
        QGraphicsEllipseItem *gdoor;
        if (door.my_rooms.size() == 0)
            qWarning() << __FUNCTION__ << "Door with no rooms" << QString::fromStdString(door.id);
        if (door.my_rooms.size() == 1) // place it to the right of the room
        {
            auto &[kr, from_room] = *(door.my_rooms.begin());
            auto room = rooms.at(from_room.room_id);
            gdoor = scene->addEllipse(-200, -200, 400, 400, QPen(QColor("lightGreen"), 50), QBrush(QColor("lightGreen")));
            if(not doors_in_rooms.contains(kr))
                gdoor->setPos(room.room_world_rect.center.x+1500, room.room_world_rect.center.y-1000);
            else
                gdoor->setPos(room.room_world_rect.center.x+1200, room.room_world_rect.center.y+1000);
            gdoor->setZValue(120);
            QFont f; f.setPointSize(180);
            auto text = scene->addText("D", f);
            text->setParentItem(gdoor);
            flip_text(text);
            text->setPos(-90,110);
            texts.push_back(text);
            doors_in_rooms.insert(kr);
            gdoors.push_back(gdoor);

            // edge
            QPointF p1(room.room_world_rect.center.x, room.room_world_rect.center.y);
            QLineF line(p1, gdoor->pos());
            auto edge = scene->addLine(line, QPen(QColor("darkGreen"), 50));
            edges.push_back(edge);
            auto text2 = scene->addText(QString::fromStdString(kd).mid(0,4) + " (" + QString::number(room.id) + ")", f);
            text2->setParentItem(edge);
            flip_text(text2);
            text2->setPos(line.center()-QPointF(300, 0));
            texts.push_back(text2);
        }
        if (door.my_rooms.size() == 2)  //place the edge between the two rooms
        {
            auto &[kfr, from_room_id] = *(door.my_rooms.begin());
            auto &[ktr, to_room_id] = *(std::next(door.my_rooms.begin()));
            Room from_room = rooms.at(from_room_id.room_id);
            Room to_room = rooms.at(to_room_id.room_id);
            gdoor = scene->addEllipse(-200, -200, 400, 400, QPen(QColor("lightGreen"), 50), QBrush(QColor("lightGreen")));
            auto r_center = (to_room.room_world_rect.center + from_room.room_world_rect.center)/2.f;
            gdoor->setPos(QPointF(r_center.x, r_center.y));
            gdoor->setZValue(120);
            QFont f; f.setPointSize(180);
            auto text = scene->addText("D", f);
            text->setParentItem(gdoor);
            flip_text(text);
            text->setPos(-90,110);
            texts.push_back(text);

            QPointF p1(from_room.room_world_rect.center.x, from_room.room_world_rect.center.y);
            QLineF line1(p1, gdoor->pos());
            auto edge1 = scene->addLine(line1, QPen(QColor("darkGreen"), 50));
            edges.push_back(edge1);
            auto text1 = scene->addText(QString::fromStdString(kd).mid(0,4) + " (" + QString::number(door.my_rooms.begin()->second.room_id) + "-" + QString::number(kfr) + ")", f);
            text1->setParentItem(edge1);
            flip_text(text1);
            text1->setPos(line1.center());
            texts.push_back(text1);
            QPointF p2(to_room.room_world_rect.center.x, to_room.room_world_rect.center.y);
            QLineF line2(gdoor->pos(), p2);
            auto edge2 = scene->addLine(line2, QPen(QColor("darkGreen"), 50));
            edges.push_back(edge2);
            auto text2 = scene->addText(QString::fromStdString(kd).mid(0,4) + " (" + QString::number(door.my_rooms.begin()->second.room_id) + "-" + QString::number(ktr) + ")", f);
            text2->setParentItem(edge2);
            flip_text(text2);
            text2->setPos(line2.center());
            texts.push_back(text2);
        }
    }
}

void Graph_Rooms::draw_all(QGraphicsScene *robot_scene, QGraphicsScene *graph_scene)
{
    draw_rooms(robot_scene);
    draw_doors(robot_scene);
    draw_nodes(graph_scene);
    draw_edges(graph_scene);
}
void Graph_Rooms::flip_text(QGraphicsTextItem *text)
{
    QTransform transform(text->transform());
    qreal m11 = transform.m11();    // Horizontal scaling
    qreal m12 = transform.m12();    // Vertical shearing
    qreal m13 = transform.m13();    // Horizontal Projection
    qreal m21 = transform.m21();    // Horizontal shearing
    qreal m22 = transform.m22();    // vertical scaling
    qreal m23 = transform.m23();    // Vertical Projection
    qreal m31 = transform.m31();    // Horizontal Position (DX)
    qreal m32 = transform.m32();    // Vertical Position (DY)
    qreal m33 = transform.m33();    // Addtional Projection Factor
    // Vertical flip
    m22 = -m22;
    // Write back to the matrix
    transform.setMatrix(m11, m12, m13, m21, m22, m23, m31, m32, m33);
    // Set the items transformation
    text->setTransform(transform);
}

void Graph_Rooms::add_door_to_current_room(const Eigen::Vector2f &p1_grid, const Eigen::Vector2f &p2_grid)
{
    // a door can only be added if it does not already exist
    // if coming from another room (current_door not -1) then, one of the computed doors must match the current_door
    // we need a == operator to compare the candidate with the new existing one
    // we only have the coordinates in the local_grid RS
    // each connecting room keeps local coordinates for the door in the Rect coord system

    // create provisional door
    uuid_t uuid;
    uuid_generate(uuid);
    char uuid_str[37];
    uuid_unparse_lower(uuid, uuid_str);
    std::string key(uuid_str);
    Graph_Rooms::Door new_door(p1_grid, p2_grid, key);  // coords in temporal grid CS.

    // check if door already exists
    auto res = std::ranges::find_if(current_room().doors_ids, [ds = doors, new_door ](auto d)mutable{ return new_door == ds.at(d);});
    if(res == std::end(current_room().doors_ids)) // Not found, go ahead to add a new door
    {
        new_door.my_rooms[current_room().id] = Door::From_Room{.room_id = current_room().id};
        doors.insert(std::pair(key, new_door));
        current_room().doors_ids.push_back(key);

        //qInfo() << __FUNCTION__ << "NEW door added" << new_door.id << " connected to current_room [" << p1.x() << p1.y() << "]"
        //        << "[" << p2.x() << p2.y() << "]" ;
    }
    //else Cannot add door to an existing door
}
Eigen::Matrix<double, 4, 3> Graph_Rooms::get_room_sides_matrix(const Room &r)
{
//    Eigen::Vector2d c1(r.room_rect.right(), r.room_rect.top());
//    Eigen::Vector2d c2(r.room_rect.right(), r.room_rect.bottom());
//    Eigen::Vector2d c3(r.room_rect.left(), r.room_rect.bottom());
//    Eigen::Vector2d c4(r.room_rect.left(), r.room_rect.top());
    Eigen::Matrix<double, 4, 3> L;
//    L << c1.y() - c2.y(), c2.x() - c1.x(), (c1.x() - c2.x()) * c1.y() + (c2.y() - c1.y()) * c1.x(),
//            c2.y() - c3.y(), c3.x() - c2.x(), (c2.x() - c3.x()) * c2.y() + (c3.y() - c2.y()) * c2.x(),
//            c3.y() - c4.y(), c4.x() - c3.x(), (c3.x() - c4.x()) * c3.y() + (c4.y() - c3.y()) * c3.x(),
//            c4.y() - c1.y(), c1.x() - c4.x(), (c4.x() - c1.x()) * c4.y() + (c1.y() - c4.y()) * c4.x();
    return L;
}
std::tuple<std::size_t, Eigen::Vector2f> Graph_Rooms::project_point_on_closest_side(const Room &r, const Eigen::Vector2f &p)
{
    cv::Point2f points[4];
    r.room_rect.points(points);
    std::vector<Eigen::Vector2f> corners = {  Eigen::Vector2f(points[0].x, points[0].y),
                                              Eigen::Vector2f(points[1].x, points[1].y),
                                              Eigen::Vector2f(points[2].x, points[2].y),
                                              Eigen::Vector2f(points[3].x, points[3].y)};

    std::vector<Eigen::ParametrizedLine<float, 2>> lines;
    std::vector<float> distances;
    for(std::size_t i = 0; i <4; i++)
    {
        lines.emplace_back(Eigen::ParametrizedLine<float, 2>::Through(corners[i], corners[(i + 1) % 4]));
        distances.push_back(lines.back().distance(p));
    }
    auto min = std::ranges::min_element(distances);
    std::size_t idx = std::distance(distances.begin(), min);
    auto proj = lines[idx].projection(p);
    return std::make_tuple(idx, proj);
}

float  Graph_Rooms::min_distance_from_point_to_closest_side(const Room &r, const Eigen::Vector2f &p) const
{
//    std::vector<Eigen::Vector2f> corners = {  Eigen::Vector2f(r.room_rect.right(), r.room_rect.top()),
//                                              Eigen::Vector2f(r.room_rect.right(), r.room_rect.bottom()),
//                                              Eigen::Vector2f(r.room_rect.left(), r.room_rect.bottom()),
//                                              Eigen::Vector2f(r.room_rect.left(), r.room_rect.top())};
//
//    std::vector<Eigen::ParametrizedLine<float, 2>> lines;
    std::vector<float> distances;
//    for(int i=0; i<4; i++)
//    {
//        lines.emplace_back(Eigen::ParametrizedLine<float, 2>::Through(corners[i], corners[(i + 1) % 4]));
//        distances.push_back(lines.back().distance(p));
//    }
    return std::ranges::min(distances);

}
void Graph_Rooms::project_doors_on_room_side(Room &r, QGraphicsScene *scene)
{
    // room in grid coordinates
    std::vector<std::string> doors_to_delete;
    for(auto &d_id: r.doors_ids)
    {
        auto &d = doors.at(d_id);
        const auto &[side1, p1] = project_point_on_closest_side(r, d.p1_in_grid);
        const auto &[side2, p2] = project_point_on_closest_side(r, d.p2_in_grid);
        if(side1 != side2)
            doors_to_delete.push_back(d_id);
        d.p1_in_grid = p1;
        d.p2_in_grid = p2;
        // compute doors coordinates in Rect RS
        Eigen::Matrix2f matrix;
        float ang = qDegreesToRadians(r.room_rect.angle);
        matrix << cos(ang) , -sin(ang) , sin(ang) , cos(ang);
        d.my_rooms.at(r.id).p1 = matrix.transpose() * ( d.p1_in_grid - Eigen::Vector2f(r.room_rect.center.x, r.room_rect.center.y));
        d.my_rooms.at(r.id).p2 = matrix.transpose() * ( d.p2_in_grid - Eigen::Vector2f(r.room_rect.center.x, r.room_rect.center.y));
    }
    for(auto &d: doors_to_delete)
        remove_door(d);
}
void Graph_Rooms::remove_doors_not_parallel_to_walls(int room_id)
{
    auto r = rooms.at(room_id);
    for(auto &d_id: r.doors_ids)
    {
        auto &d = doors.at(d_id);
        const auto &[side1, pp1] = project_point_on_closest_side(r, d.p1_in_grid);
        const auto &[side2, pp2] = project_point_on_closest_side(r, d.p2_in_grid);
        d.p1_in_grid = pp1;
        d.p2_in_grid = pp2;
        // compute doors coordinates in Rect RS
        Eigen::Matrix2f matrix;
        float ang = qDegreesToRadians(r.room_rect.angle);
        matrix << cos(ang) , -sin(ang) , sin(ang) , cos(ang);
        auto p1 = matrix.transpose() * ( d.p1_in_grid - Eigen::Vector2f(r.room_rect.center.x, r.room_rect.center.y));
        auto p2 = matrix.transpose() * ( d.p2_in_grid - Eigen::Vector2f(r.room_rect.center.x, r.room_rect.center.y));
        QLineF door(QPointF(p1.x(), p1.y()), QPointF(p2.x(), p2.y()));
        const float delta = 15;
        if((door.angle() > delta and door.angle() < M_PI / 2 - delta) //degrees
           or
           (door.angle() > M_PI/2+delta and door.angle() < M_PI)
           or
           (door.angle() > M_PI + delta and door.angle() < 3*M_PI/2 - delta)
           or
           (door.angle() > 3*M_PI/2 + delta and door.angle() < 2*M_PI - delta))
        {
            remove_door(d_id);
            qInfo() << __FUNCTION__ << "Removed door not parallel to sides" << QString::fromStdString(d_id);
        }
    }
}
void Graph_Rooms::remove_door(const std::string &id)
{
    for(const auto &[room, v] : doors.at(id).my_rooms)
        std::erase(rooms.at(room).doors_ids, id);
    doors.erase(id);
}
void Graph_Rooms::print()
{
    qInfo() << "------------------G PRINT -----------------";
    qInfo() << "    current_room:" << current_room_local;
    qInfo() << "    current_door:" << QString::fromStdString(current_door_local);
    for(auto &[k, r]  : rooms)
    {
        r.print();
        for (auto &d: r.doors_ids)
            doors.at(d).print();
    }
    qInfo() << "--------------------------------------";
}

void Graph_Rooms::merge_doors(int room_orig, const std::string &door_orig, int room_dest, const std::string &door_dest)
{
    Door::From_Room new_room; new_room.room_id = room_orig;
    doors.at(door_dest).my_rooms.insert(std::make_pair(room_orig, new_room));
    std::erase(rooms.at(room_orig).doors_ids, door_orig);
    rooms.at(room_orig).doors_ids.push_back(door_dest);
    doors.erase(door_orig);
}

std::optional<std::string> Graph_Rooms::get_door_with_only_one_room(int room) const
{
    for(const auto d_id: rooms.at(room).doors_ids)
        if(doors.at(d_id).has_only_one_room())
            return d_id;
    return {};
}

