//
// Created by pbustos on 8/12/21.
//

#include "graph_rooms.h"

void Graph_Rooms::draw_doors(const std::vector<Graph_Rooms::Door> &local_doors, QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem *> door_lines;  // to remove graph objects
    for (auto dp: door_lines) scene->removeItem(dp);
    door_lines.clear();
    for (const auto r: local_doors)
    {
        door_lines.push_back(scene->addLine(r.p1.x(), r.p1.y(), r.p2.x(), r.p2.y(), QPen(QColor("Magenta"), 50)));
        door_lines.back()->setZValue(200);
    }
}
void Graph_Rooms::draw_node(int id, QGraphicsScene *scene)
{
    //QRectF dimensions = scene->sceneRect();
    //std::mt19937 mt(std::random_device{}());
    //std::uniform_real_distribution<float> dist_x(dimensions.left()+1000, dimensions.right()-1000);
    //std::uniform_real_distribution<float> dist_y(dimensions.top()+800, dimensions.bottom()-800);
    //int x = dist_x(mt); int y = dist_y(mt);
    auto node = scene->addEllipse(0, 0, 600, 600, QPen(QColor("lightBlue"), 50), QBrush(QColor("lightBlue")));
    auto x = rooms.at(id).graph_pos.x();
    auto y = rooms.at(id).graph_pos.y();
    node->setPos(x-300, y-300);
    node->setZValue(100);
    QFont f; f.setPointSize(180);
    auto text = scene->addText("r-" + QString::number(id), f);
    text->setParentItem(node);
    flip_text(text);
    text->setPos(180, 400);
    node->setZValue(120);
}
void Graph_Rooms::draw_edge(int door_id, QGraphicsScene *scene)
{
    if(door_id == -1 or doors.at(door_id).to_rooms.size() < 2) return;
    qInfo() << __FUNCTION__ << doors.at(door_id).to_rooms.size() << "door_id" << door_id;
    int room_id_1 = *(doors.at(door_id).to_rooms.begin());
    int room_id_2 = *(++doors.at(door_id).to_rooms.begin());
    auto room_1 = rooms.at(room_id_1);
    auto room_2 = rooms.at(room_id_2);
    QPointF p1 = room_1.graph_pos;
    QPointF p2 = room_2.graph_pos;
    QLineF line(p1, p2);
    auto node = scene->addLine(line, QPen(QColor("darkGreen"), 50));
    node->setZValue(50);
    QFont f; f.setPointSize(180);
    auto text = scene->addText("d" + QString::number(door_id) + " (" + QString::number(room_id_1) + "-" + QString::number(room_id_2) +")", f);
    text->setParentItem(node);
    flip_text(text);
    text->setPos(line.center());
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
