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
//    for (std::map<CPiPuckEntity *, std::vector<std::vector<quadtree::Box>>>::const_iterator it = m_cAgVisLF.GetAgentFrontierRegions().begin();
//         it != m_cAgVisLF.GetAgentFrontierRegions().end();
//         ++it) {
//        std::vector<CColor> colors = {CColor::BROWN, CColor::CYAN, CColor::MAGENTA, CColor::YELLOW, CColor::ORANGE,
//                                      CColor::GRAY80, CColor::WHITE, CColor::BLACK, CColor::BLUE};
//        int i = 0;
//        for (auto frontierRegion: it->second) {
//            //Assign a differnet color to every frontierRegion
//            CColor color = colors[i];
//            for (auto frontier: frontierRegion) {
//                Coordinate frontierCoordinateArgos = frontier.getCenter().FromOwnToArgos();
//                CVector3 frontierCoordinate = CVector3(frontierCoordinateArgos.x, frontierCoordinateArgos.y, 0.02f);
//
//                DrawBox(frontierCoordinate, CQuaternion(), CVector3(frontier.getSize(), frontier.getSize(), 0),
//                        color);
//            }
//            i++;
//            if (i > colors.size()) i = 0;
//        }
//    }
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






    argos::LOG << "combined tree size: " << m_cAgVisLF.combinedQuadTree.size() << std::endl;

    for (std::tuple<quadtree::Box, float, double> boxesAndConfidenceAndTicks: m_cAgVisLF.combinedQuadTree) {
        quadtree::Box box = std::get<0>(boxesAndConfidenceAndTicks);
        float LConfidence = std::get<1>(boxesAndConfidenceAndTicks);
        float PConfidence = std::exp(LConfidence) / (1 + std::exp(LConfidence));
        double visitedTimeS = std::get<2>(boxesAndConfidenceAndTicks);

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
//        quadtree::Occupancy occ = quadtree::AMBIGUOUS;
//        if (LConfidence >= 0.4){
//            occ = quadtree::FREE;
//        } else if (LConfidence <= -0.85){
//            occ = quadtree::OCCUPIED;
//        }
//        if (occ == quadtree::Occupancy::OCCUPIED) {
//            color = CColor::RED;
//            DrawPolygon(pos, CQuaternion(), posVec, color, fill);
//
//        } else if (occ == quadtree::Occupancy::FREE) {
//            color = CColor::GREEN;
//            color.SetAlpha(UInt8(pheromone * 255));
//            DrawPolygon(pos, CQuaternion(), posVec, color, fill);
//        }


        color = CColor::GREEN;
        color.SetAlpha(UInt8(PConfidence * 255));
        color = color.Blend(CColor::RED);
        if(LConfidence > -0.85) color.SetAlpha(UInt8(pheromone * 255));
        DrawPolygon(pos, CQuaternion(), posVec, color, fill);

    }



    for (auto & it : m_cAgVisLF.m_tAgentBestFrontierCoordinate) {
        if (it.second == CVector3(MAXFLOAT, MAXFLOAT, 0.1f)) continue; //skip if not set
        if (it.first->GetId() == "pipuck1")
            DrawBox(it.second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::MAGENTA);
        else if (it.first->GetId() == "pipuck2")
            DrawBox(it.second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::CYAN);

    }

    for (auto & it : m_cAgVisLF.m_tAgentSubTargetCoordinate) {
        if (it.second == CVector3(MAXFLOAT, MAXFLOAT, 0.1f)) continue; //skip if not set
        if (it.first->GetId() == "pipuck1")
            DrawBox(it.second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::BROWN);
        else if (it.first->GetId() == "pipuck2")
            DrawBox(it.second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::CYAN);

    }

    for (auto & it : m_cAgVisLF.m_tAgentWallFollowingSubTargetCoordinate) {
        if (it.second == CVector3(MAXFLOAT, MAXFLOAT, 0.1f)) continue; //skip if not set
        if (it.first->GetId() == "pipuck1")
            DrawBox(it.second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::BROWN);
        else if (it.first->GetId() == "pipuck2")
            DrawBox(it.second, CQuaternion(), CVector3(0.2, 0.2, 0), CColor::CYAN);

    }

    for (auto & it : m_cAgVisLF.m_tLine) {

        std::vector<CVector3> line = it.second;

        DrawCoordinates(line, CColor::YELLOW);
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
        DrawCircle(coordinate, orientation, 0.025f, color);
    }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CAgentVisionQTUserFunctions, "agent_vision_qtuser_functions_pipuck")
