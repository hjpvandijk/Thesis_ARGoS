#include <set>
#include "agent_vision_qtuser_functions_pipuck.h"
#include "agent_vision_loop_functions_pipuck.h"

/****************************************/
/****************************************/

CAgentVisionQTUserFunctions::CAgentVisionQTUserFunctions() :
        m_cAgVisLF(dynamic_cast<CAgentVisionLoopFunctions &>(CSimulator::GetInstance().GetLoopFunctions())) {
}

/**
 * Draw the explored boxes of the quadtree in the world
 * Red if it is occupied, green if it is free
 * Also draw the 'vision' area of the agents
 */

void CAgentVisionQTUserFunctions::DrawInWorld() {
    /* Go through all the robot waypoints and draw them */
    for (std::map<CPiPuckEntity *, std::set<argos::CDegrees>>::const_iterator it = m_cAgVisLF.GetAgentFreeAngles().begin();
         it != m_cAgVisLF.GetAgentFreeAngles().end();
         ++it) {
        for (argos::CDegrees angle: it->second) {
//            CVector3 pos = it->first->GetEmbodiedEntity().GetOriginAnchor().Position;
//            CQuaternion orientation = it->first->GetEmbodiedEntity().GetOriginAnchor().Orientation;

            //Get start of the ray
            CVector3 agent_pos = m_cAgVisLF.GetAgentCoordinates().at(it->first);

            //Get the angle of the ray
            argos::CRadians angle_rad = ToRadians(angle);

            //Get the end of the ray
            CVector3 ray_end = CVector3(agent_pos.GetX() + cos(angle_rad.GetValue()),
                                        agent_pos.GetY() + sin(angle_rad.GetValue()), 0.02f);

            CRay3 ray = CRay3(agent_pos, ray_end);
            DrawRay(ray, CColor::BLUE);
        }
    }

    auto box = quadtree::Box(-5, 5, 10);
    quadtree::Quadtree *combinedTree = new quadtree::Quadtree(box);

    std::vector<std::tuple<quadtree::Box, int, double >> combinedQuadTree;

    for (std::map<CPiPuckEntity *, std::vector<std::tuple<quadtree::Box, int, double >>>::const_iterator it = m_cAgVisLF.GetQuadTree().begin();
         it != m_cAgVisLF.GetQuadTree().end();
         ++it) {
        for (std::tuple<quadtree::Box, int, double> boxAndOccupancyAndTicks: it->second) {
            quadtree::Box box = std::get<0>(boxAndOccupancyAndTicks);
            int occupancy = std::get<1>(boxAndOccupancyAndTicks);
            double visitedTimeS = std::get<2>(boxAndOccupancyAndTicks);

            quadtree::QuadNode node;
            node.coordinate = box.getCenter();
            node.occupancy = static_cast<quadtree::Occupancy>(occupancy);
            if (node.occupancy == quadtree::ANY || node.occupancy == quadtree::UNKNOWN)
                continue;
            node.visitedAtS = visitedTimeS;
            combinedTree->add(node);
        }
        std::vector<std::tuple<quadtree::Box, int, double>> boxesAndOccupancyAndTicks = combinedTree->getAllBoxes();

        combinedQuadTree = boxesAndOccupancyAndTicks;
//        if(it->first->GetId()=="pipuck1") combinedQuadTree = it->second;
    }


    argos::LOG << "combined tree size: " << combinedQuadTree.size() << std::endl;

    for (std::tuple<quadtree::Box, int, double> boxAndOccupancyAndTicks: combinedQuadTree) {
        quadtree::Box box = std::get<0>(boxAndOccupancyAndTicks);
        int occupancy = std::get<1>(boxAndOccupancyAndTicks);
        double visitedTimeS = std::get<2>(boxAndOccupancyAndTicks);

        double currentTimeS = m_cAgVisLF.globalElapsedTicks;
        double pheromone = 1.0 - std::min((currentTimeS - visitedTimeS) / 100.0, 1.0);

        Coordinate boxCenterArgos = Coordinate{box.getCenter().x, box.getCenter().y}.FromOwnToArgos();
        CVector3 pos = CVector3(boxCenterArgos.x, boxCenterArgos.y, 0.02f);

        Coordinate topLeft = Coordinate{box.left, box.top}.FromOwnToArgos();
        Coordinate topRight = Coordinate{box.getRight(), box.top}.FromOwnToArgos();
        Coordinate bottomLeft = Coordinate{box.left, box.getBottom()}.FromOwnToArgos();
        Coordinate bottomRight = Coordinate{box.getRight(), box.getBottom()}.FromOwnToArgos();

        CVector2 topLeftVec = CVector2(topLeft.x - boxCenterArgos.x, topLeft.y - boxCenterArgos.y);
        CVector2 topRightVec = CVector2(topRight.x - boxCenterArgos.x, topRight.y - boxCenterArgos.y);
        CVector2 bottomLeftVec = CVector2(bottomLeft.x - boxCenterArgos.x, bottomLeft.y - boxCenterArgos.y);
        CVector2 bottomRightVec = CVector2(bottomRight.x - boxCenterArgos.x, bottomRight.y - boxCenterArgos.y);
        std::vector<CVector2> posVec = {topLeftVec, topRightVec, bottomRightVec, bottomLeftVec};

        CColor color = CColor::GRAY80;
        bool fill = true;
        if (occupancy == quadtree::Occupancy::OCCUPIED) {
            color = CColor::RED;
            DrawPolygon(pos, CQuaternion(), posVec, color, fill);

        } else if (occupancy == quadtree::Occupancy::FREE) {
            color = CColor::GREEN;
            color.SetAlpha(UInt8(pheromone * 255));
            DrawPolygon(pos, CQuaternion(), posVec, color, fill);
        }

    }

    for (auto it = m_cAgVisLF.m_tAgentBestFrontierCoordinate.begin();
         it != m_cAgVisLF.m_tAgentBestFrontierCoordinate.end();
         ++it) {
        if (it->first->GetId() == "pipuck1")
            DrawBox(it->second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::MAGENTA);
        else if (it->first->GetId() == "pipuck2")
            DrawBox(it->second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::CYAN);

    }

    for (auto it = m_cAgVisLF.m_tAgentSubTargetCoordinate.begin();
         it != m_cAgVisLF.m_tAgentSubTargetCoordinate.end();
         ++it) {
        if (it->first->GetId() == "pipuck1")
            DrawBox(it->second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::BROWN);
        else if (it->first->GetId() == "pipuck2")
            DrawBox(it->second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::CYAN);

    }

    for (auto it = m_cAgVisLF.m_tLine.begin();
         it != m_cAgVisLF.m_tLine.end();
         ++it) {

        std::vector<CVector3> line = it->second;

        DrawCoordinates(line, CColor::YELLOW);
    }

    for (std::map<CPiPuckEntity *, CVector3>::const_iterator it = m_cAgVisLF.GetAgentCoordinates().begin();
         it != m_cAgVisLF.GetAgentCoordinates().end();
         ++it) {

        //Draw IDs
        DrawText(it->second,
                 it->first->GetId()); // text
    }


}

/****************************************/
/****************************************/

void CAgentVisionQTUserFunctions::DrawCoordinates(const std::vector<CVector3> &c_coordinates, CColor color) {
    /* Start drawing segments when you have at least two points */
    CQuaternion orientation = CQuaternion();
    for (CVector3 coordinate: c_coordinates) {
        DrawCircle(coordinate, orientation, 0.025f, color);
    }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CAgentVisionQTUserFunctions, "agent_vision_qtuser_functions_pipuck")
