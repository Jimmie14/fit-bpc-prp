#include "BaseController.h"

#include "App.h"

namespace Manhattan::Core
{
    BaseController::BaseController(const App& app) : _app(app), _node(app.GetNode()) { }
}