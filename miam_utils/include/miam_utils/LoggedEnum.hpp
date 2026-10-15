#ifndef LOGGED_STATE
#define LOGGED_STATE

    #include "miam_utils/Logger.h"

    template <typename T>
    class LoggedEnum {
        public:
            /// @brief Holder for an enum, whose every change is logged as int.
            /// @details Changing a value logs twice: old and new value at same
            ///          timestamp. This enables for simple curve display.
            LoggedEnum():
                logger_(nullptr)
            {}

            LoggedEnum(Logger *logger, std::string const& variableName, T const& value)
            {
                init(logger, variableName, value);
            }

            void init(Logger *logger, std::string const& variableName, T const& value)
            {
                logger_ = logger;
                variableId_ = logger_->registerVariable(variableName);
                value_ = value;
                logger_->log(variableId_, logger_->getElapsedTime(), static_cast<int>(value_));
            }

            void set(T const& value)
            {
                // Only log on actual update
                if (value != value_)
                {
                    if (logger_ == nullptr)
                    {
                        value_ = value;
                        return;
                    }
                    double const time = logger_->getElapsedTime();
                    logger_->log(variableId_, time, static_cast<int>(value_));
                    value_ = value;
                    logger_->log(variableId_, time, static_cast<int>(value_));
                }
            }

            T get() const
            {
                return value_;
            }
        private:
            Logger *logger_;
            uint16_t variableId_;
            T value_;
    };
#endif
