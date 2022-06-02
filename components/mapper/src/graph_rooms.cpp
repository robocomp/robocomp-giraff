//
// Created by pbustos on 8/12/21.
//

#include "graph_rooms.h"
#include <cppitertools/filter.hpp>

Graph_Rooms::Graph_Rooms()
{
    rooms.insert(std::make_pair(-1, Room(-1)));
    current_room_local = -1;
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

        auto x = r.graph_pos.x();
        auto y = r.graph_pos.y();
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
    draw_edges(scene);
}
void Graph_Rooms::draw_edges(QGraphicsScene *scene)
{
    std::vector<QGraphicsItem *> edges;
    std::vector<QGraphicsItem *> texts;
    for(const auto &e : edges)
    {
        for (auto c: e->childItems())
            scene->removeItem(c);
        scene->removeItem(e);
    }
    edges.clear();

    for(const auto &[k, r]: rooms)
        for(const auto &d_id : r.doors_ids)
        {
            const auto &d = doors.at(d_id);
            if( d.to_room != -1)
            {
                qInfo() << __FUNCTION__ << "......................." << d.id << d.to_room;
                QPointF p1 = r.graph_pos;
                QPointF p2 = rooms.at(d.to_room).graph_pos;
                QLineF line(p1, p2);
                auto edge = scene->addLine(line, QPen(QColor("darkGreen"), 50));
                edge->setZValue(50);
                QFont f;
                f.setPointSize(180);
                auto text = scene->addText("d" + QString::number(d.id) + " (" + QString::number(r.id) + "-" + QString::number(rooms.at(d.to_room).id) + ")", f);
                text->setParentItem(edge);
                flip_text(text);
                text->setPos(line.center());
                edges.push_back(edge);
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

void Graph_Rooms::add_door_to_current_room(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2)
{
    // a door can only be added if it does not already exist
    // we need a == operator to compare the candidate with the new existing one
    // we only have the coordinates in the local_grid RS
    // the door is created or updted as a node. It does not have coordinates
    // each connecting room keeps local coordinates for the door in the Rect coord system
    Graph_Rooms::Door new_door{p1, p2, (int)doors.size()};  // coords in grid RS
    auto res = std::ranges::find_if(current_room().doors_ids, [ds = doors, new_door ](int d)mutable{ return new_door == ds.at(d);});
    if(res != std::end(current_room().doors_ids)) // found
        return;  // skip
    else  // add a new door
    {
        new_door.rooms[current_room().id] = Door::From_Room{.room_id = current_room().id};
        doors.push_back(new_door);
        current_room().doors_ids.emplace_back(new_door.id);

        qInfo() << __FUNCTION__ << "NEW door " << new_door.id << " connected to current_room [" << p1.x() << p1.y() << "]"
                << "[" << p2.x() << p2.y() << "]" ;
    }

//    bool door_found = false;
//    // search for an existing door in other rooms with the same coordinates
//    for (auto &r: rooms)
//        for (auto &d_id: r.doors_ids)
//        {
//            auto &d = doors.at(d_id);
//            if (new_door == d)
//            {
//                if (r.id != room.id  and d.to_room == -1) // exists a free connecting door in a nearby room
//                {
//                    d.to_room = room.id;
//                    d.from_room = r.id;
//                    //                    new_door.to_room = r.id;
//                    //                    new_door.from_room = room.id;
//                    //                    new_door.id = d.id;
//                    room.doors_ids.emplace_back(new_door.id);
//                    qInfo() << __FUNCTION__ << "RECOGNIZED door " << new_door.id << "added to room" << room.id;
//                    d.print();
//                }
//                door_found = true;
//                break;
//            }
//    if( not door_found)
//    {
//        new_door.id = number_of_doors++;
//        new_door.from_room = current_room().id;
//        room.doors_ids.emplace_back(new_door.id);
//        qInfo() << __FUNCTION__ << "NEW door " << new_door.id << "added to room" << room.id;
//        new_door.print();
//    }
}
Eigen::Matrix<double, 4, 3> Graph_Rooms::get_room_sides_matrix(const Room &r)
{
//    Eigen::Vector2d c1(r.room_rect.right(), r.room_rect.top());
//    Eigen::Vector2d c2(r.room_rect.right(), r.room_rect.bottom());
//    Eigen::Vector2d c3(r.room_rect.left(), r.room_rect.bottom());
//    Eigen::Vector2d c4(r.room_rect.left(), r.room_rect.top());
//    Eigen::Matrix<double, 4, 3> L;
//    L << c1.y() - c2.y(), c2.x() - c1.x(), (c1.x() - c2.x()) * c1.y() + (c2.y() - c1.y()) * c1.x(),
//            c2.y() - c3.y(), c3.x() - c2.x(), (c2.x() - c3.x()) * c2.y() + (c3.y() - c2.y()) * c2.x(),
//            c3.y() - c4.y(), c4.x() - c3.x(), (c3.x() - c4.x()) * c3.y() + (c4.y() - c3.y()) * c3.x(),
//            c4.y() - c1.y(), c1.x() - c4.x(), (c4.x() - c1.x()) * c4.y() + (c1.y() - c4.y()) * c4.x();
//    return L;
}
Eigen::Vector2f Graph_Rooms::project_point_on_closest_side(const Room &r, const Eigen::Vector2f &p)
{
//    std::vector<Eigen::Vector2f> corners = {  Eigen::Vector2f(r.room_rect.right(), r.room_rect.top()),
//                                              Eigen::Vector2f(r.room_rect.right(), r.room_rect.bottom()),
//                                              Eigen::Vector2f(r.room_rect.left(), r.room_rect.bottom()),
//                                              Eigen::Vector2f(r.room_rect.left(), r.room_rect.top())};

    cv::Point2f points[4];
    r.room_rect.points(points);
    std::vector<Eigen::Vector2f> corners = {  Eigen::Vector2f(points[0].x, points[0].y),
                                              Eigen::Vector2f(points[1].x, points[1].y),
                                              Eigen::Vector2f(points[2].x, points[2].y),
                                              Eigen::Vector2f(points[3].x, points[3].y)};

    std::vector<Eigen::ParametrizedLine<float, 2>> lines;
    std::vector<float> distances;
    for(int i=0; i<4; i++)
    {
        lines.emplace_back(Eigen::ParametrizedLine<float, 2>::Through(corners[i], corners[(i + 1) % 4]));
        distances.push_back(lines.back().distance(p));
    }
    auto min = std::ranges::min_element(distances);
    int idx = std::distance(distances.begin(), min);
    auto proj = lines[idx].projection(p);
    return proj;
}

float  Graph_Rooms::min_distance_from_point_to_closest_side(const Room &r, const Eigen::Vector2f &p) const
{
//    std::vector<Eigen::Vector2f> corners = {  Eigen::Vector2f(r.room_rect.right(), r.room_rect.top()),
//                                              Eigen::Vector2f(r.room_rect.right(), r.room_rect.bottom()),
//                                              Eigen::Vector2f(r.room_rect.left(), r.room_rect.bottom()),
//                                              Eigen::Vector2f(r.room_rect.left(), r.room_rect.top())};
//
//    std::vector<Eigen::ParametrizedLine<float, 2>> lines;
//    std::vector<float> distances;
//    for(int i=0; i<4; i++)
//    {
//        lines.emplace_back(Eigen::ParametrizedLine<float, 2>::Through(corners[i], corners[(i + 1) % 4]));
//        distances.push_back(lines.back().distance(p));
//    }
//    return std::ranges::min(distances);

}
void Graph_Rooms::project_doors_on_room_side(Room &r,QGraphicsScene *scene)
{
    for(auto &d_id: r.doors_ids)
    {
        auto &d = doors.at(d_id);
        d.p1 = project_point_on_closest_side(r, d.p1);
        d.p2 = project_point_on_closest_side(r, d.p2);
        // compute doors coordinates in Rect RS
        Eigen::Matrix2f matrix;
        float ang = qDegreesToRadians(r.room_rect.angle);
        matrix << cos(ang) , -sin(ang) , sin(ang) , cos(ang);
        d.rooms.at(r.id).p1 = matrix.transpose() * ( d.p1 - Eigen::Vector2f(r.room_rect.center.x, r.room_rect.center.y));
        d.rooms.at(r.id).p2 = matrix.transpose() * ( d.p2 - Eigen::Vector2f(r.room_rect.center.x, r.room_rect.center.y));
        qInfo() << __FUNCTION__ << "door " << d.id << "in rect"
                <<  d.rooms.at(r.id).p1.x() <<  d.rooms.at(r.id).p1.y()
                <<  d.rooms.at(r.id).p2.x() <<  d.rooms.at(r.id).p2.y();

        //d.draw(scene);
    }
}