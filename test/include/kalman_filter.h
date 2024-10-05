#include "types.h"
namespace kf
{
    template<size_t DIM_X, size_t DIM_Z>
    class KalmanFilter
    {
    public:
        KalmanFilter()
        {

        }

        ~KalmanFilter()
        {

        }
        
        virtual Vector<DIM_X>& vecX() { return m_vecX; }
        virtual const Vector<DIM_X>& vecX() const { return m_vecX; }

        virtual Matrix<DIM_X, DIM_X>& matP() { return m_matP; }
        virtual const Matrix<DIM_X, DIM_X>& matP() const { return m_matP; }

        template<typename PredictionModelCallback>
        void predictEkf(PredictionModelCallback predictionModelFunc, const Matrix<DIM_X, DIM_X>& matJacobF, const Matrix<DIM_X, DIM_X>& matQ)
        {
            m_vecX = predictionModelFunc(m_vecX);
            m_matP = matJacobF * m_matP * matJacobF.transpose() + matQ;
        }

        template<typename MeasurementModelCallback>
        void correctEkf(MeasurementModelCallback measurementModelFunc, const Vector<DIM_Z>& vecZ, 
            const Matrix<DIM_Z, DIM_Z>& matR, const Matrix<DIM_Z, DIM_X>& matJcobH)
        {
            const Matrix<DIM_X, DIM_X> matI{ Matrix<DIM_X, DIM_X>::Identity() };
            const Matrix<DIM_Z, DIM_Z> matSk{ matJcobH * m_matP * matJcobH.transpose() + matR }; // Innovation covariance
            const Matrix<DIM_X, DIM_Z> matKk{ m_matP * matJcobH.transpose() * matSk.inverse() }; // Kalman Gain

            m_vecX = m_vecX + matKk * (vecZ - measurementModelFunc(m_vecX));
            m_matP = (matI - matKk * matJcobH) * m_matP;
        }

    protected:
        Vector<DIM_X> m_vecX{ Vector<DIM_X>::Zero() };
        Matrix<DIM_X, DIM_X> m_matP{ Matrix<DIM_X, DIM_X>::Zero() };
    };

}