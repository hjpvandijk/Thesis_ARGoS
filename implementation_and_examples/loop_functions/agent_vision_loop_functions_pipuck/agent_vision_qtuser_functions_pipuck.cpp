#include "agent_vision_qtuser_functions_pipuck.h"
#include "agent_vision_loop_functions_pipuck.h"

/****************************************/
/****************************************/

CAgentVisionQTUserFunctions::CAgentVisionQTUserFunctions() :
        m_cAgVisLF(dynamic_cast<CAgentVisionLoopFunctions &>(CSimulator::GetInstance().GetLoopFunctions())) {
}

Coordinate getRealCoordinateFromIndex(int x, int y) {
    //Get the real world coordinates from the matrix coordinates
    double x_real = -5 + x * 0.2;
    double y_real = -5 + y * 0.2;
    return {x_real + 0.2/2, y_real + 0.2/2};
}

/**
 * Draw the explored boxes of the quadtree in the world
 * Red if it is occupied, green if it is free
 * Also draw the 'vision' area of the agents
 */

void CAgentVisionQTUserFunctions::DrawInWorld() {
    /* Go through all the robot waypoints and draw them */





//    argos::LOG << "combined tree size: " << combinedQuadTree.size() << std::endl;

    for (int i = 0; i < m_cAgVisLF.coverageMatrixWidth; i++) {
        for (int j = 0; j < m_cAgVisLF.coverageMatrixHeight; j++) {
//            argos::LOG << "i: " << i << " j: " << j << std::endl;
            double visitedTimeS = m_cAgVisLF.coverageMatrix[i][j];
            double currentTimeS = m_cAgVisLF.globalElapsedTicks;
            double pheromone = visitedTimeS == -1 ? 0 : 1.0 - std::min((currentTimeS - visitedTimeS) / 100.0, 1.0);
            if (pheromone == 0) continue;
//            argos::LOG << "visitedTimeS: " << visitedTimeS << " currentTimeS: " << currentTimeS << " pheromone: " << pheromone << std::endl;

//            Coordinate cellCoordinate = m_cAgVisLF.combinedCoverageMatrix.getRealCoordinateFromIndex(i, j).FromOwnToArgos();
            Coordinate cellCoordinate = getRealCoordinateFromIndex(i, j);
//            argos::LOG << "cellCoordinate: " << cellCoordinate.x << " " << cellCoordinate.y << std::endl;
//
//            CVector3 pos = CVector3(i, j, 0.02f);
//            CColor color = CColor::GREEN;
//            color.SetAlpha(UInt8(pheromone * 255));
//            DrawCircle(pos, CQuaternion(), 0.1f, color);
//        }


            Coordinate cellCenterArgos = Coordinate{cellCoordinate.x, cellCoordinate.y}.FromOwnToArgos();

            CVector3 pos = CVector3(cellCenterArgos.x, cellCenterArgos.y, 0.02f);

            auto coverageResolution = m_cAgVisLF.coverageMatrixResolution;

            Coordinate topLeft = Coordinate{cellCoordinate.x - coverageResolution/2, cellCoordinate.y + coverageResolution/2}.FromOwnToArgos();
            Coordinate topRight = Coordinate{cellCoordinate.x + coverageResolution/2, cellCoordinate.y + coverageResolution/2}.FromOwnToArgos();
            Coordinate bottomLeft = Coordinate{cellCoordinate.x - coverageResolution/2, cellCoordinate.y - coverageResolution/2}.FromOwnToArgos();
            Coordinate bottomRight = Coordinate{cellCoordinate.x + coverageResolution/2, cellCoordinate.y - coverageResolution/2}.FromOwnToArgos();

            CVector2 topLeftVec = CVector2(topLeft.x - cellCenterArgos.x, topLeft.y - cellCenterArgos.y);
            CVector2 topRightVec = CVector2(topRight.x - cellCenterArgos.x, topRight.y - cellCenterArgos.y);
            CVector2 bottomLeftVec = CVector2(bottomLeft.x - cellCenterArgos.x, bottomLeft.y - cellCenterArgos.y);
            CVector2 bottomRightVec = CVector2(bottomRight.x - cellCenterArgos.x, bottomRight.y - cellCenterArgos.y);
            std::vector<CVector2> posVec = {topLeftVec, topRightVec, bottomRightVec, bottomLeftVec};

            CColor color;
            bool fill = true;
            color = CColor::GREEN;
            color.SetAlpha(UInt8(pheromone * 255));
            DrawPolygon(pos, CQuaternion(), posVec, color, fill);
        }

    }

    for (int i = 0; i < m_cAgVisLF.obstacleMatrixWidth; i++) {
        for (int j = 0; j < m_cAgVisLF.obstacleMatrixHeight; j++) {
//            argos::LOG << "i: " << i << " j: " << j << std::endl;
            double visitedTimeS = m_cAgVisLF.obstacleMatrix[i][j];
            double currentTimeS = m_cAgVisLF.globalElapsedTicks;
            double pheromone = visitedTimeS == -1 ? 0 : 1.0 - std::min((currentTimeS - visitedTimeS) / 100.0, 1.0);
            if (pheromone == 0) continue;
//            argos::LOG << "visitedTimeS: " << visitedTimeS << " currentTimeS: " << currentTimeS << " pheromone: " << pheromone << std::endl;

//            Coordinate cellCoordinate = m_cAgVisLF.combinedCoverageMatrix.getRealCoordinateFromIndex(i, j).FromOwnToArgos();
            Coordinate cellCoordinate = getRealCoordinateFromIndex(i, j);
//
//            CVector3 pos = CVector3(i, j, 0.02f);
//            CColor color = CColor::GREEN;
//            color.SetAlpha(UInt8(pheromone * 255));
//            DrawCircle(pos, CQuaternion(), 0.1f, color);
//        }


            Coordinate cellCenterArgos = Coordinate{cellCoordinate.x, cellCoordinate.y}.FromOwnToArgos();

            CVector3 pos = CVector3(cellCenterArgos.x, cellCenterArgos.y, 0.04f);

            auto obstacleResolution = m_cAgVisLF.obstacleMatrixResolution;

            Coordinate topLeft = Coordinate{cellCoordinate.x - obstacleResolution / 2, cellCoordinate.y + obstacleResolution / 2}.FromOwnToArgos();
            Coordinate topRight = Coordinate{cellCoordinate.x + obstacleResolution / 2, cellCoordinate.y + obstacleResolution / 2}.FromOwnToArgos();
            Coordinate bottomLeft = Coordinate{cellCoordinate.x - obstacleResolution / 2, cellCoordinate.y - obstacleResolution / 2}.FromOwnToArgos();
            Coordinate bottomRight = Coordinate{cellCoordinate.x + obstacleResolution / 2, cellCoordinate.y - obstacleResolution / 2}.FromOwnToArgos();

            CVector2 topLeftVec = CVector2(topLeft.x - cellCenterArgos.x, topLeft.y - cellCenterArgos.y);
            CVector2 topRightVec = CVector2(topRight.x - cellCenterArgos.x, topRight.y - cellCenterArgos.y);
            CVector2 bottomLeftVec = CVector2(bottomLeft.x - cellCenterArgos.x, bottomLeft.y - cellCenterArgos.y);
            CVector2 bottomRightVec = CVector2(bottomRight.x - cellCenterArgos.x, bottomRight.y - cellCenterArgos.y);
            std::vector<CVector2> posVec = {topLeftVec, topRightVec, bottomRightVec, bottomLeftVec};

            CColor color;
            bool fill = true;
            color = CColor::RED;
            color.SetAlpha(UInt8(pheromone * 255));
            DrawPolygon(pos, CQuaternion(), posVec, color, fill);
        }

    }

    for (auto &it:  m_cAgVisLF.m_tAgentFrontierRegions) {
        if (it.first->GetId() == "pipuck1") {
            std::vector<CColor> colors = {CColor::BROWN, CColor::CYAN, CColor::MAGENTA, CColor::YELLOW, CColor::ORANGE,
                                          CColor::GRAY80, CColor::WHITE, CColor::BLACK, CColor::BLUE};
            int i = 0;
            for (auto frontierRegion: it.second) {
                //Assign a differnet color to every frontierRegion
                CColor color = colors[i];
                for (auto frontier: frontierRegion) {
                    Coordinate frontierCoordinateArgos = frontier.FromOwnToArgos();
                    CVector3 frontierCoordinate = CVector3(frontierCoordinateArgos.x, frontierCoordinateArgos.y, 0.05f);

                    DrawBox(frontierCoordinate, CQuaternion(),
                            CVector3(m_cAgVisLF.coverageMatrixResolution, m_cAgVisLF.coverageMatrixResolution, 0),
                            color);
                }
                i++;
                if (i > colors.size()) i = 0;
            }
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
