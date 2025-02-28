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

double calculatePheromone(double visitedTime, double PConfidence, double currentTime) {
    double pheromoneFactor = 1.0 - std::min((currentTime - visitedTime) / 300.0, (1.0 - 0.05));
    double pheromone = pheromoneFactor * (PConfidence-0.5) + 0.5;
    //This makes sure that a value once set to occupied or free, will not be changed to ambiguous again due to evaporation.
    //So we assume that if a cell is occupied or free, it will stay that way, albeit with a lower confidence.
    if (PConfidence <= 0.4) pheromone = pheromoneFactor * (PConfidence - 0.4) + 0.4;
//    if (PConfidence >= 0.6) pheromone = pheromoneFactor * (PConfidence - 0.6) + 0.6;
    return pheromone;
}

void CAgentVisionQTUserFunctions::DrawInWorld() {

    for (auto & m_tCell : m_cAgVisLF.m_tNeighborPairs) {
        if (m_tCell.first->GetId() != "pipuck1") continue;
        for (std::tuple<Coordinate, Coordinate> neighborPair: m_tCell.second) {
            Coordinate neighbor1 = std::get<0>(neighborPair);
            Coordinate neighbor2 = std::get<1>(neighborPair);

            Coordinate neighbor1argos = neighbor1.FromOwnToArgos();
            Coordinate neighbor2argos = neighbor2.FromOwnToArgos();

            CVector3 pos1 = CVector3(neighbor1argos.x, neighbor1argos.y, 0.02f);
            CVector3 pos2 = CVector3(neighbor2argos.x, neighbor2argos.y, 0.02f);
            auto ray = CRay3(pos1, pos2);
            DrawRay(ray, CColor::BLUE, 20.0f);
        }
    }

    for (auto & m_tRoute : m_cAgVisLF.m_tAgentRoute){
        for (std::tuple<Coordinate, Coordinate> route: m_tRoute.second) {
            Coordinate start = std::get<0>(route);
            Coordinate end = std::get<1>(route);


            Coordinate startArgos = start.FromOwnToArgos();
            Coordinate endArgos = end.FromOwnToArgos();

            CVector3 pos1 = CVector3(startArgos.x, startArgos.y, 0.02f);
            CVector3 pos2 = CVector3(endArgos.x, endArgos.y, 0.02f);
            auto ray = CRay3(pos1, pos2);
            DrawRay(ray, CColor::YELLOW, 10.0f);
        }
    }

//    for (auto &agentFrontiers: m_cAgVisLF.m_tAgentFrontiers) {
//        if (agentFrontiers.first->GetId() != "pipuck6") continue;
//        for (auto [frontier, pheromone]: agentFrontiers.second) {
//            Coordinate frontierCoordinateArgos = frontier.getCenter().FromOwnToArgos();
//            CVector3 frontierCoordinate = CVector3(frontierCoordinateArgos.x, frontierCoordinateArgos.y, 0.02f);
//
//            DrawBox(frontierCoordinate, CQuaternion(), CVector3(frontier.getSize(), frontier.getSize(), 0),
//                    CColor::PURPLE);
//        }
//    }

////    /* Go through all the robot waypoints and draw them */
//    for (auto & m_tAgentFrontierRegions : m_cAgVisLF.m_tAgentFrontierRegions) {
//        std::vector<CColor> colors = {CColor::BROWN, CColor::CYAN, CColor::MAGENTA, CColor::YELLOW, CColor::ORANGE,
//                                      CColor::GRAY80, CColor::WHITE, CColor::BLACK, CColor::BLUE, CColor::GRAY10, CColor::GRAY20, CColor::GRAY30, CColor::GRAY40, CColor::GRAY50, CColor::GRAY60, CColor::GRAY70, CColor::GRAY80, CColor::GRAY90};
//        if (m_tAgentFrontierRegions.first->GetId() != "pipuck1") continue;
//        int i = 0;
//        for (auto frontierRegions: m_tAgentFrontierRegions.second) {
//            //Assign a differnet color to every frontierRegion
//            CColor color = colors[i];
//            for (auto [frontier, pheromone]: frontierRegions) {
//                Coordinate frontierCoordinateArgos = frontier.getCenter().FromOwnToArgos();
//                CVector3 frontierCoordinate = CVector3(frontierCoordinateArgos.x, frontierCoordinateArgos.y, 0.02f);
//
//                DrawBox(frontierCoordinate, CQuaternion(), CVector3(frontier.getSize(), frontier.getSize(), 0),
//                        color);
////                DrawText(frontierCoordinate - CVector3(0.05,-0.05,0), std::to_string(frontier.getCenter().x) + " " + std::to_string(frontier.getCenter().y), CColor::BLACK);
//
//            }
//            i++;
//            if (i > colors.size()) i = 0;
//
//        }
//    }

for (auto & agentheading : m_cAgVisLF.m_tAgentHeadings) {
    CRadians heading = m_cAgVisLF.m_tAgentHeadings[agentheading.first];
    CVector3 pos = m_cAgVisLF.m_tAgentCoordinates[agentheading.first];
    CVector3 ray_end = CVector3(pos.GetX() + cos(heading.GetValue()),
                                pos.GetY() + sin(heading.GetValue()), 0.02f);
    CRay3 ray = CRay3(pos, ray_end);
    DrawRay(ray, CColor::RED, 10.0f);
}




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
            CVector3 ray_end = CVector3(agent_pos.GetX() + 0.28*cos(angle_rad.GetValue()),
                                        agent_pos.GetY() + 0.28*sin(angle_rad.GetValue()), 0.02f);

            CRay3 ray = CRay3(agent_pos, ray_end);
            DrawRay(ray, CColor::BLUE);
        }
    }






    argos::LOG << "combined tree size: " << m_cAgVisLF.combinedQuadTree.size() << std::endl;

    for (std::tuple<quadtree::Box, double> boxesAndPheromone: m_cAgVisLF.combinedQuadTree) {
        quadtree::Box box = std::get<0>(boxesAndPheromone);
        float pheromone = std::get<1>(boxesAndPheromone);

        double currentTimeS = m_cAgVisLF.globalElapsedTime;


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
        color.SetAlpha(UInt8(pheromone * 255));
        color = color.Blend(CColor::RED);
//        if(LConfidence > -0.85) color.SetAlpha(UInt8(pheromone * 255));
        DrawPolygon(pos, CQuaternion(), posVec, color, fill);
        DrawPolygon(pos, CQuaternion(), posVec, CColor::BLACK, false);

//        DrawText(pos, std::to_string(pheromone), CColor::BLACK);
//        //Also write the coordinates in the box
//        DrawText(pos - CVector3(0.05,-0.05,0), std::to_string(box.getCenter().x) + " " + std::to_string(box.getCenter().y), CColor::BLACK);
//        auto p_free_threshold = 0.7;
//        auto p_occupied_threshold= 0.3;
//        if (pheromone >= p_free_threshold) {
//            DrawText(pos - CVector3(0.05,-0.05,0), "FREE", CColor::BLACK);
//        } else if (pheromone <= p_occupied_threshold) {
//            DrawText(pos - CVector3(0.05,-0.05,0), "OCCUPIED", CColor::BLACK);
//        } else {
//            DrawText(pos - CVector3(0.05,-0.05,0), "AMBIGUOUS", CColor::BLACK);
//        }
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
//#ifdef BATTERY_MANAGEMENT_ENABLED
        float batteryLevel = m_cAgVisLF.m_tAgentBatteryLevels.at(it.first);
        DrawText(it.second + CVector3(0.1,0.1,0.1), std::to_string(batteryLevel) + '%', CColor::BLACK);
//#endif
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
