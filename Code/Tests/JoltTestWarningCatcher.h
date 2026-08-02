#pragma once

#include <AzCore/Debug/TraceMessageBus.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace JoltPhysics
{
    //! Collects gem warnings so a test can assert that a diagnostic actually fired
    //! (and keeps them out of the test output while connected). Connect it around the
    //! call under test by giving it a scope.
    class JoltWarningCatcher : public AZ::Debug::TraceMessageBus::Handler
    {
    public:
        JoltWarningCatcher()
        {
            BusConnect();
        }
        ~JoltWarningCatcher() override
        {
            BusDisconnect();
        }

        bool OnPreWarning(const char* window, const char*, int, const char*, const char* message) override
        {
            if (window && AZStd::string_view(window) == "JoltPhysics")
            {
                m_warnings.push_back(message ? message : "");
            }
            return true; // handled; do not print
        }

        //! Errors are collected the same way, for tests whose subject is a diagnostic
        //! firing - a malformed asset, say - where the error is the expected outcome
        //! rather than a failure of the test.
        bool OnPreError(const char* window, const char*, int, const char*, const char* message) override
        {
            if (window && AZStd::string_view(window) == "JoltPhysics")
            {
                m_warnings.push_back(message ? message : "");
                return true;
            }
            return false;
        }

        bool ContainsWarningWith(AZStd::string_view substring) const
        {
            for (const AZStd::string& warning : m_warnings)
            {
                if (warning.contains(substring))
                {
                    return true;
                }
            }
            return false;
        }

        AZStd::vector<AZStd::string> m_warnings;
    };
} // namespace JoltPhysics
