#include "BaseController.h"

#include "App.h"

namespace Manhattan::Core
{
    BaseController::BaseController(const App& app) : _app(app), _node(app.GetNode()) { }

    void BaseController::Enable() {
        if (_enabled) return;
        _enabled = true;

        OnEnable();
    }

    void BaseController::Disable() {
        if (!_enabled) return;
        _enabled = false;

        OnDisable();
    }
}
