#pragma once

#include <cstdint>
#include "esp_err.h"
#include "esp_log.h"
#include <exception>
#include <type_traits>
#include <functional>

// #include <optional>

#if defined(__cpp_exceptions) || defined(__EXCEPTIONS)
#define HAVE_EXCEPTIONS (1)
#else
#define HAVE_EXCEPTIONS (0)
#endif

namespace esp::err {

    /**
     * @brief A std::exception which also has a descriptive type() string; it is expected that type() returns 
     * the exception's class name so that RTTI is not needed.
     * 
     */
    struct typed_exception : public std::exception {

        inline typed_exception(const char* const what = nullptr) : m_what {what} {

        }

        inline typed_exception(const typed_exception& other) = default;

        inline typed_exception(typed_exception&& other) = default;

        inline typed_exception& operator =(const typed_exception& other) = default;
        inline typed_exception& operator =(typed_exception&& other) = default;    

        virtual const char* type() const noexcept = 0;

        inline const char* what() const noexcept override {
            return m_what;
        }

        protected:
            const char* m_what;
    };

    // template<class SELF>
    // struct typed_exception_base : public typed_exception {

    //     using self_t = SELF;

    //     const char* type() const noexcept override {
    //         return self_t::TYPE;
    //     }
    // };

    struct esp_error : public typed_exception {

        /**
         * @brief Throws an \c esp_error if the given \p result is not \c ESP_OK.
         * 
         * @param[in] result the \c esp_err_t to check 
         * @param[in] msg (optional) the message to include in the exception
         * @throws a corresponding \c esp_error if \p result is not \c ESP_OK
         */
        static inline void if_failed(const esp_err_t result, const char* const msg = nullptr) noexcept(false);

        /**
         * @brief Returns the given pointer if it is \e not \c null, throws an \c esp_error of \c ESP_FAIL
         * if it is \c null.
         * 
         * @tparam T a pointer type
         * @param[in] ptr pointer value to check
         * @param[in] msg (optional) message to include in the exception
         * @return T the given \p ptr, never \c null
         */
        template<typename T, std::enable_if_t<std::is_pointer_v<T>,bool> = true>
        static inline T if_null(T ptr, const char* const msg = nullptr) noexcept(false) {
            return if_null(ptr, ESP_FAIL, msg);
        }

        /**
         * @brief 
         * 
         * @param[in] value boolean required to be true
         * @param[in] msg (optional) message to include in the exception
         * @throws an \c esp_error of \c ESP_FAIL if the given \p value is \c false
         */
        static inline void if_false(const bool value, const char* const msg = nullptr);

        template<typename T>
        static inline T if_not(T value, T expected, const char* const msg = nullptr);

        /**
         * @brief Returns the given pointer if it is \e not \c null, throws an \c esp_error if it is \c null. 
         * 
         * @tparam T a pointer type
         * @param[in] ptr pointer value to check
         * @param[in] err the \c esp_err_t value for the exception
         * @param[in] msg (optional) message to include in the exception
         * @return T the given \p ptr, never \c null
         */
        template<typename T, std::enable_if_t<std::is_pointer_v<T>,bool> = true>
        static inline T if_null(T ptr, const esp_err_t err, const char* const msg = nullptr);    

        /**
         * @brief Returns the given pointer if it is \e not \c null, throws an \c esp_error of \c ESP_NO_MEM
         * if it is \c null.
         * 
         * @tparam T a pointer type
         * @param[in] ptr pointer value to check
         * @param[in] msg (optional) message to include in the exception
         * @return T the given \p ptr
         */
        template<typename T, std::enable_if_t<std::is_pointer_v<T>,bool> = true>
        static inline T if_alloc_failed(T ptr, const char* const msg = nullptr);



        inline esp_error(const esp_err_t err = ESP_FAIL, const char* what = nullptr) noexcept :
            typed_exception(what),
            m_err {err} {

        }

        inline esp_error(const esp_error& other) noexcept = default;

        inline esp_error(esp_error&& other) noexcept = default;

        constexpr operator esp_err_t() const noexcept {
            return m_err;
        }

        constexpr esp_error& operator =(const esp_err_t err) noexcept {
            m_err = err;
            return *this;
        }

        constexpr esp_error& operator =(const esp_error& other) noexcept = default;

        constexpr esp_error& operator =(esp_error&& other) noexcept = default;

        const char* type() const noexcept override {
            return "esp_error";
        }

        constexpr esp_err_t err() const noexcept {
            return m_err;
        }

        inline const char* err_name() const noexcept {
            return esp_err_to_name(m_err);
        }

        protected:
            esp_err_t m_err;

    };

    struct timeout_exception : public esp_error {

        inline timeout_exception(const timeout_exception& other) noexcept = default;
        inline timeout_exception(timeout_exception&& other) noexcept = default;
        inline timeout_exception(const char* const what = nullptr) noexcept :
            esp_error(ESP_ERR_TIMEOUT,what) {

            }

        constexpr timeout_exception& operator =(const timeout_exception& other) noexcept = default;

        constexpr timeout_exception& operator =(timeout_exception&& other) noexcept = default;        

        const char* type() const noexcept override {
            return "timeout_exception";
        }
    };

    struct initialization_error : public typed_exception {

        inline initialization_error(const char* const what = nullptr) : typed_exception(what) {

        }

        inline initialization_error(const initialization_error& other) = default;

        inline initialization_error(initialization_error&& other) = default;

        inline initialization_error& operator =(const initialization_error& other) = default;
        inline initialization_error& operator =(initialization_error&& other) = default;    

        const char* type() const noexcept override {
            return "initialization_error";
        }

    };

    // #if !HAVE_EXCEPTIONS
    #include "esp_rom_sys.h"
    // #endif // HAVE_EXCEPTIONS

    #ifdef __cpp_rtti
        #include <typeinfo>

        #define TYPE_NAME(x) (typeid(x).name())

    #else
        #define TYPE_NAME(x) "<unknown>"
    #endif // __cpp_rtti

    // __attribute__((cold)) implies "unlikely" for calling branches

    template<typename E>
    inline void __attribute__((cold)) logEx(const E& exception) noexcept {

        if constexpr (std::is_base_of_v<std::exception,E>) {
            const char* const msg = exception.what();
            if constexpr (std::is_base_of_v<typed_exception,E>) {
                const char* const type = exception.type();
                if constexpr (std::is_base_of_v<esp_error,E>) {
                    esp_rom_printf("Error: %s %d %s, what: %s\r\n", type, exception.err(), exception.err_name(), msg);                    
                } else {
                    esp_rom_printf("Error: %s, what: %s\r\n", type, msg);
                }
            } else {
                // Here, we only know that exception is statically typed std::exception&
                #if defined(__cpp_rtti)
                    // See if we can narrow its type down using RTTI
                    const esp_error* const ee = dynamic_cast<const esp_error*>(&exception);
                    if(ee) {
                        esp_rom_printf("Error: %s %d %s, what: %s\r\n", ee->type(), ee->err(), ee->err_name(), ee->what());  
                    } else {
                        const typed_exception* const te = dynamic_cast<const typed_exception*>(&exception);
                        if(te) {
                            esp_rom_printf("Error: %s, what: %s\r\n", te->type(), te->what());
                        } else {
                            // It's neither a typed_exception nor esp_error... but still a std::exception. Treat it as a std::exception.
                            // const std::exception* const se = dynamic_cast<const std::exception*>(&exception);
                            // if(se) {
                            esp_rom_printf("Error: An exception occurred, type: %s, what: %s\r\n", TYPE_NAME(exception), exception.what());
                            // } else {
                            //     esp_rom_printf("Error: An exception occurred, type: %s\r\n", TYPE_NAME(exception));
                            // }
                        }
                    }
                #else     
                    // Treat it as a std::exception.       
                    esp_rom_printf("Error: An exception occurred, type: %s, what: %s\r\n", TYPE_NAME(exception), msg);
                #endif
            }
        } else {
            // exception is statically typed as something not derived from std::exception... could be anything.
            esp_rom_printf("Error: An exception occurred, type: %s\r\n", TYPE_NAME(exception));        
        }    
    }

    template<typename E>
    inline __attribute__((noreturn,cold)) bool doThrow(const E& exception) noexcept(!HAVE_EXCEPTIONS) {
        #if HAVE_EXCEPTIONS
            throw exception;
        #else
            logEx(exception);
            abort();
        #endif
    }

    inline __attribute__((noreturn,cold)) bool throwIllegalArg(const char* const msg = nullptr) {
        doThrow(esp_error {ESP_ERR_INVALID_ARG, msg});
    }

    inline __attribute__((noreturn,cold)) bool throwOOM(const char* const msg = nullptr) {
        doThrow(esp_error {ESP_ERR_NO_MEM, msg});
    }


    struct esp_result_t {

        constexpr esp_result_t(const esp_err_t result = ESP_OK) : result {result} {

        }

        constexpr esp_result_t(const esp_result_t& other) = default;

        constexpr bool ok() const {
            return result == ESP_OK;
        }

        constexpr bool error() const {
            return result != ESP_OK;
        }

        /**
         * @brief 
         * 
         * @return true if ::ok()
         * @return false if not ::ok()
         */
        constexpr explicit operator bool() const {
            return ok();
        }


        /**
         * @brief Ensures that this result is \c ESP_OK , throwing an ::esp_error exception 
         * or \c abort() ing if it's not.
         * 
         * @param[in] failMsg optional message to include in the exception
         * @return const esp_result_t& this ::esp_result_t
         * @throws ::esp_error if this result is not \c ESP_OK , or \c abort() s if exceptions are disabled.
         */
        inline const esp_result_t& onErrorThrow(const char* const failMsg = nullptr) const noexcept(false) {
            doValidate(failMsg);
            return *this;
        }

        inline const esp_result_t& throwOnError(const char* const failMsg = nullptr) const noexcept(false) {
            return onErrorThrow(failMsg);
        }

        /**
         * @brief Ensures that this result is \c ESP_OK , throwing an ::esp_error exception 
         * or \c abort() ing if it's not.
         * 
         * @return const esp_result_t& this ::esp_result_t
         * @throws ::esp_error if this result is not \c ESP_OK , or \c abort() s if exceptions are disabled.
         */
        // inline const esp_result_t& validate(const char* const failMsg = nullptr) const noexcept(false) {
        //     doValidate(failMsg);
        //     return *this;
        // }

        // /**
        //  * @brief same as #validate() const above.
        //  */
        // inline esp_result_t& validate() noexcept(false) {
        //     doValidate();
        //     return *this;
        // }

        // /**
        //  * @brief same as #validate() const above: Requires this result to be \c ESP_OK ,
        //  * throwing ::esp_error or aborting otherwise.
        //  */
        // inline const esp_result_t& reqOk(const char* const failMsg = nullptr) const noexcept(false) {
        //     return validate();
        // }

        // /**
        //  * @brief same as #validate() above: Requires this result to be \c ESP_OK ,
        //  * throwing ::esp_error or aborting otherwise.
        //  */
        // inline esp_result_t& reqOk() noexcept(false) {
        //     return validate();
        // }    

        // constexpr operator esp_err_t() const {
        //     return result;
        // }

        constexpr esp_err_t value() const {
            return result;
        }

        inline const char* msg() const {
            return esp_err_to_name(result);
        }

        constexpr esp_result_t& operator =(const esp_err_t e) noexcept {
            this->result = e;
            return *this;
        }

        constexpr esp_result_t& operator =(const esp_result_t& other) = default;

        constexpr bool operator ==(const esp_result_t& other) const {
            return this->result == other.result;
        }

        constexpr bool operator !=(const esp_result_t& other) const {
            return this->result != other.result;
        }

        constexpr bool operator ==(const esp_err_t other) const {
            return this->result == other;
        }

        constexpr bool operator !=(const esp_err_t other) const {
            return this->result != other;
        }    

        private:
            esp_err_t result;

            inline void doValidate(const char* const failMsg = nullptr) const noexcept(false) {
                if(error()) {
                    if(result == ESP_ERR_TIMEOUT) {
                        doThrow(timeout_exception {failMsg});
                    } else {
                        doThrow(esp_error {result, failMsg});
                    }
                }
            }

    };

    constexpr bool operator==(const esp_err_t e, const esp_result_t& r) noexcept {
        return e == r.value();
    }

    constexpr bool operator!=(const esp_err_t e, const esp_result_t& r) noexcept {
        return e != r.value();
    }



    // template<typename R>
    // struct esp_result_value_t {

    //     esp_result_t result {};
    //     std::optional<R> value {};

    //     constexpr esp_result_value_t(const R& value) :
    //         value {value} {

    //     }

    //     constexpr esp_result_value_t(const esp_result_t& result) :
    //         result {result} {

    //         }

    //     inline R onErrorThrow(const char* const msg = nullptr) {
    //         result.onErrorThrow(msg);
    //         return value.value();
    //     }
    // };

    // template<typename R>
    // inline esp_result_value_t<R> ok(const R& value) {
    //     return esp_result_value_t<R> {value};
    // }

    // template<typename R>
    // inline esp_result_value_t<R> fail(const esp_err_t error) {
    //     return esp_result_value_t<R> {esp_result_t {error}};
    // }



    template<auto& F, typename... Args>
    requires requires (Args&&...rgs) {{std::invoke(F,std::forward<Args>(rgs)...)} -> std::convertible_to<esp_result_t>;}
    void ex(Args&&... args) {
        const esp_result_t r = std::invoke(F,std::forward<Args>(args)...);
        r.throwOnError();
    }

    inline void esp_error::if_failed(const esp_err_t result, const char* const msg) {
        const esp_result_t r {result};
        r.throwOnError(msg);
    }

    template<typename T, std::enable_if_t<std::is_pointer_v<T>,bool>>
    inline T esp_error::if_null(T ptr, const esp_err_t err, const char* const msg) {
        if(ptr == nullptr) {
            doThrow(esp_error {err, msg != nullptr ? msg : "unexpected null ptr"});
        }
        return ptr;
    }

    template<typename T, std::enable_if_t<std::is_pointer_v<T>,bool>>
    inline T esp_error::if_alloc_failed(T ptr, const char* const msg) {
        if(ptr == nullptr) {
            throwOOM(msg != nullptr ? msg : "allocation failed");
        }
        return ptr;
    }

    inline void esp_error::if_false(const bool value, const char* const msg) {
        if(!value) {
            doThrow(esp_error {ESP_FAIL, msg != nullptr ? msg : "required condition was false"});
        }
    }

    template<typename T>
    inline T esp_error::if_not(T value, T expected, const char* const msg) {
        if(!(value == expected)) {
            doThrow(esp_error {ESP_FAIL, msg != nullptr ? msg : "value not as expected"});
        } else {
            return value;
        }
    }

} // namespace esp::err