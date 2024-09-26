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





//    argos::LOG << "combined tree size: " << combinedQuadTree.size() << std::endl;

    for (std::tuple<quadtree::Box, int, double> boxAndOccupancyAndTicks: m_cAgVisLF.combinedQuadTree) {
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

        CColor color;
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



    for (auto & it : m_cAgVisLF.m_tAgentBestFrontierCoordinate) {
        if (it.first->GetId() == "pipuck1")
            DrawBox(it.second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::MAGENTA);
        else if (it.first->GetId() == "pipuck2")
            DrawBox(it.second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::CYAN);

    }

    for (const auto & it : m_cAgVisLF.GetAgentCoordinates()) {

        //Draw IDs
        DrawText(it.second,
                 it.first->GetId()); // text
    }




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
