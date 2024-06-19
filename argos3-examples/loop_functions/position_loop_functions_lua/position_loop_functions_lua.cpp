#include "position_loop_functions_lua.h"
#include "agent.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/core/wrappers/lua/lua_controller.h>

/****************************************/
/****************************************/

CPositionLoopFunctions::CPositionLoopFunctions()
   {
}

void setLuaAgentPosition(lua_State* L, double x, double y) {
    // Ensure myAgent is on the Lua stack
    lua_getglobal(L, "myAgent");
    if (lua_isnil(L, -1)) {
        std::cerr << "myAgent is nil" << std::endl;
        lua_pop(L, 1); // Remove the nil value
        return;
    }

    // Get the setPosition method from the myAgent userdata or object
    lua_getfield(L, -1, "setPosition");
    if (!lua_isfunction(L, -1)) {
        std::cerr << "setPosition is not a function" << std::endl;
        lua_pop(L, 2); // Remove the non-function setPosition and myAgent
        return;
    }

    // Push myAgent as the first argument (self)
    lua_getglobal(L, "myAgent");

    // Push the x and y arguments
    lua_pushnumber(L, x);
    lua_pushnumber(L, y);

    // Call the function with 3 arguments and no return values
    if (lua_pcall(L, 3, 0, 0) != LUA_OK) {
        std::cerr << "Error calling setPosition: " << lua_tostring(L, -1) << std::endl;
        lua_pop(L, 1); // Pop the error message
    }

    // Remove myAgent from the stack
    lua_pop(L, 1);
}


/****************************************/
/****************************************/

void CPositionLoopFunctions::PreStep() {
    /* Logic to update the position of the pipucks */
    CSpace::TMapPerType &m_cPipucks = GetSpace().GetEntitiesByType("pipuck");


    for (CSpace::TMapPerType::iterator it = m_cPipucks.begin();
         it != m_cPipucks.end();
         ++it) {
        /* Get handle to pi-puck entity and controller */
        CPiPuckEntity &cPiPuck = *any_cast<CPiPuckEntity *>(it->second);
        CLuaController &cController = dynamic_cast<CLuaController &>(cPiPuck.GetControllableEntity().GetController());

        /* Get the position of the pi-puck on the ground as a CVector2 */
        CVector2 cPos;
        cPos.Set(cPiPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                 cPiPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

//        cController.agentObject->setPosition(cPos.GetX(), cPos.GetY());

        //Get the myAgent variable from the lua state and make it into an agent pointer
        lua_State *L = cController.GetLuaState();
//        lua_getglobal(L, "myAgent");
//        agent *thisagent = (agent *) lua_touserdata(L, -1);
//        agent *thisagent = *static_cast<agent**>(luaL_checkudata(L, -1, s));
//        lua_pop(L, 1);
//        argos::LOG << "Agent id from loop: " << thisagent->getId() << std::endl;
//        thisagent->setPosition(cPos.GetX(), cPos.GetY());
//        lua_close(L);

        setLuaAgentPosition(L, cPos.GetX(), cPos.GetY());





    }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CPositionLoopFunctions, "position_loop_functions_lua")
