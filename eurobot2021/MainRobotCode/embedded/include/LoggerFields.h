/// \file LoggerFields.h
/// \brief Declaration of the log fields and related function.
/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3

#ifndef LOGGER_FIELDS_H
     #define LOGGER_FIELDS_H

    ///< Global includes
    #include <miam_utils/miam_utils.h>

    #include <algorithm>

    // Logger-related variables.
    /// \brief List of values to log.
    /// \details Elements of this list determine both the enum values and the header string.
    /// \note Values should be uppercase and start with LOGGER_
    #define LOGGER_VALUES(f) \
        f(LOGGER_TIME)   \
        f(LOGGER_COMMAND_VELOCITY_RIGHT)  \
        f(LOGGER_COMMAND_VELOCITY_LEFT)   \
        f(LOGGER_MOTOR_POSITION_RIGHT)  \
        f(LOGGER_MOTOR_POSITION_LEFT)  \
        f(LOGGER_ENCODER_RIGHT)  \
        f(LOGGER_ENCODER_LEFT)  \
        f(LOGGER_CURRENT_POSITION_X)  \
        f(LOGGER_CURRENT_POSITION_Y)  \
        f(LOGGER_CURRENT_POSITION_THETA) \
        f(LOGGER_TARGET_POSITION_X)  \
        f(LOGGER_TARGET_POSITION_Y)  \
        f(LOGGER_TARGET_POSITION_THETA) \
        f(LOGGER_TARGET_LINEAR_VELOCITY) \
        f(LOGGER_TARGET_ANGULAR_VELOCITY) \
        f(LOGGER_TRACKING_LONGITUDINAL_ERROR) \
        f(LOGGER_TRACKING_TRANSVERSE_ERROR) \
        f(LOGGER_TRACKING_ANGLE_ERROR) \
        f(LOGGER_RAIL_POSITION) \
        f(LOGGER_CURRENT_VELOCITY_LINEAR) \
        f(LOGGER_CURRENT_VELOCITY_ANGULAR) \
        f(LOGGER_LINEAR_P_I_D_CORRECTION) \
        f(LOGGER_ANGULAR_P_I_D_CORRECTION) \
        f(LOGGER_RAIL_P_I_D_CORRECTION) \
        f(LOGGER_DETECTION_COEFF) \
        f(LOGGER_LIDAR_N_POINTS) \
        f(LOGGER_RANGE_RIGHT) \
        f(LOGGER_RANGE_LEFT) \

    #define GENERATE_ENUM(ENUM) ENUM,

    ///< Logger field enum.
    typedef enum{
        LOGGER_VALUES(GENERATE_ENUM)
    }LoggerFields;


    // Create header string.
    #define GENERATE_STRING(STRING) #STRING,
    static const char *LOGGER_HEADERS[] = {
        LOGGER_VALUES(GENERATE_STRING)
        NULL
    };

    // Process input string to go from enum name to string name.
    inline std::string enumToHeaderString(std::string const& s)
    {
        std::string outputString = s;
        // Remove logger_ prefix.
        outputString = outputString.substr(7);
        // Put it in lowercase.
        std::transform(outputString.begin(), outputString.end(), outputString.begin(), ::tolower);
        //Iterate over string, looking for underscores.
        for(unsigned int i = 0; i < outputString.length(); i++)
        {
            if(outputString[i] == '_')
            {
                outputString.erase(i, 1);
                outputString[i] = std::toupper(outputString[i]);
            }
        }
        return outputString;
    }
    /// \brief Create a comma-separated formated string list from LoggerFields enum.
    /// \details This string is meant to be given to the Logger object. It consits of the LoggerFields names, lowercase
    ///          and without the LOGGER_ prefix.
    /// \return A string of all headers, comma-separated. The returned string should be freed when no longer needed.
    static inline std::string getHeaderStringList()
    {
        std::string headerString = enumToHeaderString(LOGGER_HEADERS[0]);
        int i = 1;
        while(LOGGER_HEADERS[i] != NULL)
        {
            headerString = headerString + "," + enumToHeaderString(LOGGER_HEADERS[i]);
            i++;
        }
        return headerString;
    }

 #endif
