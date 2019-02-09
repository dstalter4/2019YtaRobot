////////////////////////////////////////////////////////////////////////////////
/// @file   RobotUtils.hpp
/// @author David Stalter
///
/// @details
/// Contains declarations of utility macros/routines for the robot code.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef ROBOTUTILS_HPP
#define ROBOTUTILS_HPP

// SYSTEM INCLUDES
#include <cstdarg>                              // for va_*
#include <cstdio>                               // for printf
#include <iostream>                             // for cout

// C INCLUDES
// (none)

// C++ INCLUDES
// (none)

// MACROS
#define ASSERT(condition)                                   \
    do                                                      \
    {                                                       \
        if (!(condition))                                   \
        {                                                   \
            std::cout << "Robot code ASSERT!" << std::endl; \
            std::cout << "File: " << __FILE__ << std::endl; \
            std::cout << "Line: " << __LINE__ << std::endl; \
            assert(false);                                  \
        }                                                   \
    }                                                       \
    while (false);

#define STRINGIFY(s) #s

#define DISABLE_WARNING(flag)                               \
    _Pragma("GCC diagnostic push")                          \
    _Pragma(STRINGIFY(GCC diagnostic ignored flag))

#define ENABLE_WARNING(flag)                                \
    _Pragma("GCC diagnostic pop")

////////////////////////////////////////////////////////////////
/// @namespace RobotUtils
///
/// Namespace that contains utility functions for supporting
/// robot development.
///
////////////////////////////////////////////////////////////////
namespace RobotUtils
{
    static const bool DEBUG_PRINTS = true;
    
    ////////////////////////////////////////////////////////////////
    /// @method YtaRobot::DisplayMessage
    ///
    /// Displays a message to the RioLog as long as debug prints are
    /// enabled.
    ///
    ////////////////////////////////////////////////////////////////
    inline void DisplayMessage(const char * pMessage)
    {
        if (DEBUG_PRINTS)
        {
            std::cout << pMessage << std::endl;
        }
    }
    
    ////////////////////////////////////////////////////////////////
    /// @method YtaRobot::DisplayMessage
    ///
    /// Displays a message to the RioLog as long as debug prints are
    /// enabled.
    ///
    ////////////////////////////////////////////////////////////////
    inline void DisplayFormattedMessage(const char * pMessage, ...)
    {
        if (DEBUG_PRINTS)
        {
            va_list argPtr;
            va_start(argPtr, pMessage);
            
            // It's ok to pass the pointer here instead of a formatted
            // string since the callers of this function are well defined.
            DISABLE_WARNING("-Wformat-nonliteral")
            printf(pMessage, argPtr);
            ENABLE_WARNING("-Wformat-nonliteral")
            
            va_end(argPtr);
        }
    }
    
}

#endif // ROBOTUTILS_HPP
