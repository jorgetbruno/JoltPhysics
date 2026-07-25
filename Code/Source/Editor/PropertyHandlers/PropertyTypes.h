#pragma once

namespace JoltPhysics::Editor
{
    //! Installs the gem's property handlers for the physics types AzFramework reflects
    //! but supplies no editor widget for (collision layer, collision group).
    //! Safe to call when another physics gem already registered them: handlers whose
    //! name is already claimed are skipped.
    void RegisterPropertyTypes();

    //! Removes and destroys whatever RegisterPropertyTypes installed.
    void UnregisterPropertyTypes();
} // namespace JoltPhysics::Editor
