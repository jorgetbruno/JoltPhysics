#include <Editor/PropertyHandlers/PropertyTypes.h>

#include <AzCore/std/containers/vector.h>
#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI.h>

#include <Editor/PropertyHandlers/CollisionGroupWidget.h>
#include <Editor/PropertyHandlers/CollisionLayerWidget.h>

namespace JoltPhysics::Editor
{
    namespace
    {
        AZStd::vector<AzToolsFramework::PropertyHandlerBase*>& GetRegisteredHandlers()
        {
            static AZStd::vector<AzToolsFramework::PropertyHandlerBase*> handlers;
            return handlers;
        }

        //! These are default handlers keyed by type, so registering one whose name is
        //! already taken would put two handlers in contention for the same property.
        //! PhysX ships identical handlers under the same AzFramework names, so in a
        //! project running both gems whichever activates first wins and this one is
        //! skipped rather than fighting it.
        bool IsHandlerNameAvailable(AZ::u32 handlerName, const AZ::Uuid& handledType)
        {
            AzToolsFramework::PropertyHandlerBase* existingHandler = nullptr;
            AzToolsFramework::PropertyTypeRegistrationMessageBus::BroadcastResult(
                existingHandler,
                &AzToolsFramework::PropertyTypeRegistrationMessages::ResolvePropertyHandler,
                handlerName,
                handledType);
            return existingHandler == nullptr;
        }

        template<typename HandlerType, typename PropertyType>
        void RegisterHandler()
        {
            auto* handler = aznew HandlerType();

            if (!IsHandlerNameAvailable(handler->GetHandlerName(), azrtti_typeid<PropertyType>()))
            {
                delete handler;
                return;
            }

            AzToolsFramework::PropertyTypeRegistrationMessageBus::Broadcast(
                &AzToolsFramework::PropertyTypeRegistrationMessages::RegisterPropertyType, handler);
            GetRegisteredHandlers().push_back(handler);
        }
    } // namespace

    void RegisterPropertyTypes()
    {
        RegisterHandler<CollisionLayerWidget, AzPhysics::CollisionLayer>();
        RegisterHandler<CollisionGroupWidget, AzPhysics::CollisionGroups::Id>();
    }

    void UnregisterPropertyTypes()
    {
        for (AzToolsFramework::PropertyHandlerBase* handler : GetRegisteredHandlers())
        {
            AzToolsFramework::PropertyTypeRegistrationMessageBus::Broadcast(
                &AzToolsFramework::PropertyTypeRegistrationMessages::UnregisterPropertyType, handler);
            delete handler;
        }
        GetRegisteredHandlers().clear();
        // Function-local statics destruct after the test allocator's leak check runs;
        // release the buffer now so a registration cycle leaves nothing outstanding.
        GetRegisteredHandlers().shrink_to_fit();
    }
} // namespace JoltPhysics::Editor
