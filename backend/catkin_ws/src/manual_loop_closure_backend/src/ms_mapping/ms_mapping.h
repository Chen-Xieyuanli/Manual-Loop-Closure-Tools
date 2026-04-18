#ifndef MANUAL_LOOP_CLOSURE_BACKEND_MS_MAPPING_H_
#define MANUAL_LOOP_CLOSURE_BACKEND_MS_MAPPING_H_

#include <iostream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;

class XYPriorFactor : public NoiseModelFactor1<Pose3>
{
    using Base = NoiseModelFactor1<Pose3>;
    using This = XYPriorFactor;

    Point2 measured_;

#ifdef OptionalNone
    using OptionalJacobian = OptionalMatrixType;
#else
    using OptionalJacobian = typename NoiseModelFactor1<Pose3>::template OptionalMatrix<Pose3>;
#endif

public:
    XYPriorFactor() = default;

    XYPriorFactor(Key key, const Point2 &xy, const SharedNoiseModel &model)
        : Base(model, key), measured_(xy)
    {
    }

    void print(const std::string &s = "", const KeyFormatter &keyFormatter = DefaultKeyFormatter) const override
    {
        std::cout << (s.empty() ? "" : s + " ") << "XYPriorFactor on "
                  << keyFormatter(this->key()) << "\n"
                  << "  measurement: [" << measured_.x() << ", " << measured_.y() << "]\n";
        noiseModel_->print("  noise model: ");
    }

    bool equals(const NonlinearFactor &expected, double tol = 1e-9) const override
    {
        const This *other = dynamic_cast<const This *>(&expected);
        return other != nullptr && Base::equals(*other, tol) &&
               traits<Point2>::Equals(measured_, other->measured_, tol);
    }

    Vector evaluateError(
        const Pose3 &pose,
        OptionalJacobian H =
#ifdef OptionalNone
            OptionalNone
#else
            OptionalJacobian()
#endif
            ) const override
    {
        const Point3 t = pose.translation();

        if (H)
        {
            const Matrix3 R = pose.rotation().matrix();
            Matrix26 J;
            J.setZero();
            J(0, 3) = R(0, 0);
            J(0, 4) = R(0, 1);
            J(0, 5) = R(0, 2);
            J(1, 3) = R(1, 0);
            J(1, 4) = R(1, 1);
            J(1, 5) = R(1, 2);
            (*H) = J;
        }

        Vector2 err;
        err << (t.x() - measured_.x()), (t.y() - measured_.y());
        return err;
    }

    const Point2 &measured() const { return measured_; }
};

#endif  // MANUAL_LOOP_CLOSURE_BACKEND_MS_MAPPING_H_
