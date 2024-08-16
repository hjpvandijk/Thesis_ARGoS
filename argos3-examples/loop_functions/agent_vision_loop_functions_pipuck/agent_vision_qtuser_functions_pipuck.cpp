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

//    for(CAgentVisionLoopFunctions::TCoordinateMap ::const_iterator it = m_cAgVisLF.GetOtherAgentCoordinates().begin();
//        it != m_cAgVisLF.GetOtherAgentCoordinates().end();
//        ++it){
//        DrawCoordinates(it->second, CColor::BLUE);
//    }

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
            node.visitedAtS = visitedTimeS;
            combinedTree->add(node);
        }
        std::vector<std::tuple<quadtree::Box, int, double>> boxesAndOccupancyAndTicks = combinedTree->getAllBoxes();

        combinedQuadTree = boxesAndOccupancyAndTicks;
    }


    for (std::tuple<quadtree::Box, int, double> boxAndOccupancyAndTicks: combinedQuadTree) {
        quadtree::Box box = std::get<0>(boxAndOccupancyAndTicks);
        int occupancy = std::get<1>(boxAndOccupancyAndTicks);
        double visitedTimeS = std::get<2>(boxAndOccupancyAndTicks);

        double currentTimeS = m_cAgVisLF.globalElapsedTicks;
        double pheromone = 1.0 - std::min((currentTimeS - visitedTimeS) / 100.0, 1.0);
//            argos::LOG << "Pheromone: " << agent_ticks << " - " << ticks << " = " << agent_ticks - ticks << " = " << pheromone << std::endl;


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
        bool fill = false;
        if (occupancy == quadtree::Occupancy::OCCUPIED) {
            color = CColor::RED;
            fill = true;
            DrawPolygon(pos, CQuaternion(), posVec, color, fill);
//                DrawCircle(pos, CQuaternion(), 0.25f, CColor::MAGENTA);
        } else if (occupancy == quadtree::Occupancy::FREE) {
            color = CColor::GREEN;
//                argos::LOG << "Green: " << pheromone*255 << std::endl;
            color.SetAlpha(pheromone * 255);
//                color.SetAlpha(127);
            fill = true;
            DrawPolygon(pos, CQuaternion(), posVec, color, fill);

        }


    }


    for (auto it = m_cAgVisLF.m_tAgentBestFrontierCoordinate.begin();
         it != m_cAgVisLF.m_tAgentBestFrontierCoordinate.end();
         ++it) {
        if (it->first->GetId() == "pipuck1")
            DrawBox(it->second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::MAGENTA);
        else
        if (it->first->GetId() == "pipuck2")
            DrawBox(it->second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::CYAN);

    }

    for (std::map<CPiPuckEntity *, CVector3>::const_iterator it = m_cAgVisLF.GetAgentCoordinates().begin();
         it != m_cAgVisLF.GetAgentCoordinates().end();
         ++it) {
////        DrawBox(it->second, CQuaternion(), 0.1f, CColor::GRAY50);
////        DrawBox(it->second, CQuaternion(), CVector3(4, 4, 0), CColor::GRAY80);
//        DrawCircle(it->second, CQuaternion(), 0.5f, CColor::GRAY80);

        //Draw IDs
        DrawText(it->second,
                 it->first->GetId()); // text
    }
//    for (CAgentVisionLoopFunctions::TCoordinateMap::const_iterator it = m_cAgVisLF.GetObjectCoordinates().begin();
//         it != m_cAgVisLF.GetObjectCoordinates().end();
//         ++it) {
//        DrawCoordinates(it->second, CColor::RED);
//    }




}

/****************************************/
/****************************************/

void CAgentVisionQTUserFunctions::DrawCoordinates(const std::vector<CVector3> &c_coordinates, CColor color) {
    /* Start drawing segments when you have at least two points */
    CQuaternion orientation = CQuaternion();
    for (CVector3 coordinate: c_coordinates) {
        DrawCircle(coordinate, orientation, 0.1f, color);
    }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CAgentVisionQTUserFunctions, "agent_vision_qtuser_functions_pipuck")
