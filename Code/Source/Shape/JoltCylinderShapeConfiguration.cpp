#include <Shape/JoltCylinderShapeConfiguration.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltCylinderShapeConfiguration::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<Physics::ShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltCylinderShapeConfiguration, Physics::ShapeConfiguration>()
                ->Version(1)
                ->Field("Height", &JoltCylinderShapeConfiguration::m_height)
                ->Field("Radius", &JoltCylinderShapeConfiguration::m_radius)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltCylinderShapeConfiguration>("Cylinder Configuration", "Cylinder shape properties")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltCylinderShapeConfiguration::m_height,
                        "Height", "Total height of the cylinder along its local Z axis.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltCylinderShapeConfiguration::m_radius,
                        "Radius", "Radius of the cylinder.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ;
            }
        }
    }
} // namespace JoltPhysics
