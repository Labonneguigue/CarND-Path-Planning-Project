#ifndef PID_H
#define PID_H

/** Proportional Integral Derivative Controller implementation
 *
 * @tparam T type of the floating point arguments and class members
 */
template<typename T>
class PID {
public:

    /**
     * Default Constructor
     */
    PID()
    : mKp()
    , mKi()
    , mKd()
    , mProportionalError()
    , mIntegralError()
    , mDerivativeError()
    {}

    /**
     * Default Destructor.
     */
    virtual ~PID()
    {}

    /**
     * Initialize PID.
     */
    inline void init(T Kp, T Ki, T Kd){
        mKp = Kp;
        mKi = Ki;
        mKd = Kd;
    }

    /**
     * Reset each of the errors
     */
    inline void reset()
    {
        mProportionalError = T(0.0);
        mDerivativeError = T(0.0);
        mIntegralError = T(0.0);
    }

    /**
     * Update the PID error variables given cross track error.
     */
    inline void updateError(T cte){
        mIntegralError = cte - mProportionalError;
        mProportionalError = cte;
        mDerivativeError += cte;
    }

    /**
     * Calculate the total PID error.
     */
    inline T totalError(){
        return ( - (mKp * mProportionalError)
                 - (mKi * mIntegralError)
                 - (mKd * mDerivativeError) );
    }

private:

    T mKp; ///< Proportional coefficient
    T mKi; ///< Integral coefficient
    T mKd; ///< Derivative coefficient

    T mProportionalError; ///< Error itself
    T mIntegralError; ///< Integral Error since start of error updates
    T mDerivativeError; ///< Derivative of the error over each subsequent frame
    
};

#endif /* PID_H */
