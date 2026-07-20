#pragma once

#include <AzCore/Serialization/SerializeContext.h>

namespace JoltPhysics
{
    namespace Internal
    {
        //! Reflects T with the serialize context only if it has not been reflected yet.
        //! Used for AzPhysics/AzFramework configuration classes that may already have
        //! been reflected by another module in the application.
        template<typename T>
        void ReflectOnce(AZ::ReflectContext* context)
        {
            if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                if (serializeContext->FindClassData(azrtti_typeid<T>()) == nullptr)
                {
                    T::Reflect(serializeContext);
                }
            }
        }
    } // namespace Internal
} // namespace JoltPhysics
