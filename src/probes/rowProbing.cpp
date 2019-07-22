//
// Created by Jiatian Sun on 2019-07-08.
//
#include <mitsuba/render/probe.h>
#include <mitsuba/render/scene.h>
#include <mitsuba/render/probe.h>
#include <iostream>
#include <mitsuba/core/warp.h>
#include <mitsuba/render/emitter.h>
#include <mitsuba/render/medium.h>
#include <mitsuba/core/track.h>
#include <mitsuba/hw/renderer.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/fstream.h>
#include <mitsuba/core/mstream.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/core/timer.h>
#include <mitsuba/render/mipmap.h>
#include <mitsuba/hw/gpuprogram.h>
#include <mitsuba/hw/gputexture.h>
#include <math.h>

MTS_NAMESPACE_BEGIN

/*!\plugin{perspective}{Perspective pinhole camera}
 * \order{1}
 * \parameters{
 *     \parameter{toWorld}{\Transform\Or\Animation}{
 *	      Specifies an optional camera-to-world transformation.
 *        \default{none (i.e. camera space $=$ world space)}
 *     }
 *     \parameter{focalLength}{\String}{
 *         Denotes the camera's focal length specified using
 *         \code{35mm} film equivalent units. See the main
 *         description for further details.
 *         \default{\code{50mm}}
 *     }
 *     \parameter{fov}{\Float}{
 *         An alternative to \code{focalLength}:
 *         denotes the camera's field of view in degrees---must be
 *         between 0 and 180, excluding the extremes.
 *     }
 *     \parameter{fovAxis}{\String}{
 *         When the parameter \code{fov} is given (and only then),
 *         this parameter further specifies the image axis, to
 *         which it applies.
 *         \begin{enumerate}[(i)]
 *             \item \code{\textbf{x}}: \code{fov} maps to the
 *                 \code{x}-axis in screen space.
 *             \item \code{\textbf{y}}: \code{fov} maps to the
 *                 \code{y}-axis in screen space.
 *             \item \code{\textbf{diagonal}}: \code{fov}
 *                maps to the screen diagonal.
 *             \item \code{\textbf{smaller}}: \code{fov}
 *                maps to the smaller dimension
 *                (e.g. \code{x} when \code{width}<\code{height})
 *             \item \code{\textbf{larger}}: \code{fov}
 *                maps to the larger dimension
 *                (e.g. \code{y} when \code{width}<\code{height})
 *         \end{enumerate}
 *         The default is \code{\textbf{x}}.
 *     }
 *     \parameter{shutterOpen, shutterClose}{\Float}{
 *         Specifies the time interval of the measurement---this
 *         is only relevant when the scene is in motion.
 *         \default{0}
 *     }
 *     \parameter{nearClip, farClip}{\Float}{
 *         Distance to the near/far clip
 *         planes.\default{\code{near\code}-\code{Clip=1e-2} (i.e.
 *         \code{0.01}) and {\code{farClip=1e4} (i.e. \code{10000})}}
 *     }
 * }
 * \renderings{
 * \rendering{The material test ball viewed through a perspective pinhole
 * camera. Everything is in sharp focus.}{sensor_perspective}
 * \medrendering{A rendering of the Cornell box}{sensor_perspective_2}
 * }
 *
 * This plugin implements a simple idealizied perspective camera model, which
 * has an infinitely small aperture. This creates an infinite depth of field,
 * i.e. no optical blurring occurs. The camera is can be specified to move during
 * an exposure, hence temporal blur is still possible.
 *
 * By default, the camera's field of view is specified using a 35mm film
 * equivalent focal length, which is first converted into a diagonal field
 * of view and subsequently applied to the camera. This assumes that
 * the film's aspect ratio matches that of 35mm film (1.5:1), though the
 * parameter still behaves intuitively when this is not the case.
 * Alternatively, it is also possible to specify a field of view in degrees
 * along a given axis (see the \code{fov} and \code{fovAxis} parameters).
 *
 * The exact camera position and orientation is most easily expressed using the
 * \code{lookat} tag, i.e.:
 * \begin{xml}
 * <sensor type="perspective">
 *     <transform name="toWorld">
 *         <!-- Move and rotate the camera so that looks from (1, 1, 1) to (1, 2, 1)
 *              and the direction (0, 0, 1) points "up" in the output image -->
 *         <lookat origin="1, 1, 1" target="1, 2, 1" up="0, 0, 1"/>
 *     </transform>
 * </sensor>
 * \end{xml}
 */

class RowProbing : public BasicProbe{
public:
    typedef TSpectrum<half, SPECTRUM_SAMPLES> SpectrumHalf;
    typedef TMIPMap<Spectrum, SpectrumHalf> MIPMap;
    RowProbing(const Properties &props) : BasicProbe(props) {
        m_colTransform = 0;
        m_type = "row";
    }

    RowProbing(Stream *stream, InstanceManager *manager)
            : BasicProbe(stream, manager) {
        m_rowTransform = stream->readInt();
        m_colTransform = stream->readInt();
    }

    void serialize(Stream *stream, InstanceManager *manager) const {
        BasicProbe::serialize(stream, manager);
        stream->writeInt(m_rowTransform);
        stream->writeInt(m_colTransform);
    }

    void readFromProjector(Stream *stream) {
        BasicProbe::readFromProjector(stream);
    }

    void readFromCamera(Stream *stream) {
        BasicProbe::readFromCamera(stream);
    }

    void configure() {
        BasicProbe::configure();
    }

    /**
     * \brief Compute the directional sensor response function
     * of the camera multiplied with the cosine foreshortening
     * factor associated with the image plane
     *
     * \param d
     *     A normalized direction vector from the aperture position to the
     *     reference point in question (all in local camera space)
     */
    inline Float importance(const Vector &d, EImportanceType type) const {
        /* How is this derived? Imagine a hypothetical image plane at a
           distance of d=1 away from the pinhole in emitter space.
           Then the visible rectangular portion of the plane has the area
              A = (2 * tan(0.5 * xfov in radians))^2 / aspect
           Since we allow crop regions, the actual visible area is
           potentially reduced:
              A' = A * (cropX / filmX) * (cropY / filmY)
           Perspective transformations of such aligned rectangles produce
           an equivalent scaled (but otherwise undistorted) rectangle
           in screen space. This means that a strategy, which uniformly
           generates samples in screen space has an associated area
           density of 1/A' on this rectangle.
           To compute the solid angle density of a sampled point P on
           the rectangle, we can apply the usual measure conversion term:
              d_omega = 1/A' * distance(P, origin)^2 / cos(theta)
           where theta is the angle that the unit direction vector from
           the origin to P makes with the rectangle. Since
              distance(P, origin)^2 = Px^2 + Py^2 + 1
           and
              cos(theta) = 1/sqrt(Px^2 + Py^2 + 1),
           we have
              d_omega = 1 / (A' * cos^3(theta))
        */

        Float cosTheta = Frame::cosTheta(d);

        /* Check if the direction points behind the emitter */
        if (cosTheta <= 0)
            return 0.0f;

        /* Compute the position on the plane at distance 1 */
        Float invCosTheta = 1.0f / cosTheta;
        Point2 p(d.x * invCosTheta, d.y * invCosTheta);

        /* Check if the point lies inside the chosen crop rectangle */
        switch(type){
            case ECAM:{
                if (!m_cameraImageRect->contains(p))
                    return 0.0f;
                return m_cameraNormalization * invCosTheta
                       * invCosTheta * invCosTheta;
            }
            case EPROJ:{
                if (!m_projectorImageRect->contains(p))
                    return 0.0f;
                return m_projectorNormalization * invCosTheta
                       * invCosTheta * invCosTheta;
            }
            default:
                return 0.f;
        }
    }

    Spectrum sampleProjectorPosition(PositionSamplingRecord &pRec, Sampler *sampler,
                                     const Point2 &sample, const Point2 *extra) const {
        const Transform &trafo = m_projectorWorldTransform->eval(pRec.time);
        pRec.p = trafo(Point(0.0f));
        pRec.n = trafo(Vector(0.0f, 0.0f, 1.0f));
        pRec.pdf = 1.0f;
        pRec.measure = EDiscrete;

        return Spectrum(1.0f);
    }

    Spectrum sampleCameraPosition(PositionSamplingRecord &pRec,
                                  const Point2 &sample, const Point2 *extra) const {
        const Transform &trafo = m_cameraWorldTransform->eval(pRec.time);
        pRec.p = trafo(Point(0.0f));
        pRec.n = trafo(Vector(0.0f, 0.0f, 1.0f));
        pRec.pdf = 1.0f;
        pRec.measure = EDiscrete;

        return Spectrum(1.0f);
    }

    Float pdfProjectorPosition(const PositionSamplingRecord &pRec) const {
        return (pRec.measure == EDiscrete) ? 1.0f  : 0.0f;
    }

    Float pdfCameraPosition(const PositionSamplingRecord &pRec) const {
        return (pRec.measure == EDiscrete) ? 1.0f : 0.0f;
    }

    Spectrum evalProjectorPosition(const PositionSamplingRecord &pRec) const {
        return Spectrum((pRec.measure == EDiscrete) ? 1.0f : 0.0f);
    }

    Spectrum evalCameraPosition(const PositionSamplingRecord &pRec) const {
        return Spectrum((pRec.measure == EDiscrete) ? 1.0f : 0.0f);
    }

    Spectrum sampleProjectorDirection(DirectionSamplingRecord &dRec,
                                      PositionSamplingRecord &pRec, Sampler *sampler,
                                      const Point2 &sample, const Point2 *extra) const {
        const Transform &trafo = m_projectorWorldTransform->eval(pRec.time);
        Point samplePos(sample.x, sample.y, 0.0f);

        if (extra) {
            Point2 resamplePosition((int)std::ceil(m_projectorResolution.x * sampler->next1D()),
                                    extra->y);
            /* The caller wants to condition on a specific pixel position */
            samplePos.x = (resamplePosition.x + sample.x) * m_projectorInvResolution.x;
            samplePos.y = (resamplePosition.y + sample.y) * m_projectorInvResolution.y;
        }

        Point2 uvSample(samplePos.x * m_projectorResolution.x, samplePos.y * m_projectorResolution.y);
        int xUnitTransform = m_colTransform % m_projectorSize.x;
        int xFinalTransform = xUnitTransform < 0 ? xUnitTransform + m_projectorSize.x: xUnitTransform;
        int yUnitTransform = m_rowTransform % m_projectorSize.y;
        int yFinalTransform = yUnitTransform < 0 ? yUnitTransform + m_projectorSize.y: yUnitTransform;

        int xPixelPos = (math::floorToInt(uvSample.x) + xFinalTransform)% m_projectorSize.x;
        int yPixelPos = (math::floorToInt(uvSample.y) + yFinalTransform)% m_projectorSize.y;
        Float samplePosRelativeToPixelx = uvSample.x-floor(uvSample.x);
        Float samplePosRelativeToPixely = uvSample.y-floor(uvSample.y);


        samplePos = Point((xPixelPos + samplePosRelativeToPixelx) * m_projectorInvResolution.x,
                          (yPixelPos + samplePosRelativeToPixely) * m_projectorInvResolution.y,
                          0.0f);

        pRec.uv = Point2(samplePos.x * m_projectorResolution.x,
                         samplePos.y * m_projectorResolution.y);

        /* Compute the corresponding position on the
           near plane (in local camera space) */
        Point nearP = m_sampleToProjector(samplePos);

        /* Turn that into a normalized ray direction */
        Vector d = normalize(Vector(nearP));
        dRec.d = trafo(d);
        dRec.measure = ESolidAngle;
        dRec.pdf = (Float)m_projectorResolution.y * m_projectorNormalization / (d.z * d.z * d.z);

        return Spectrum(m_scale/(Float)m_projectorResolution.y);
    }

    Spectrum sampleCameraDirection(DirectionSamplingRecord &dRec,
                                   PositionSamplingRecord &pRec,
                                   const Point2 &sample, const Point2 *extra) const {
        const Transform &trafo = m_cameraWorldTransform->eval(pRec.time);


        Point samplePos(sample.x, sample.y, 0.0f);

        if (extra) {
            /* The caller wants to condition on a specific pixel position */
            samplePos.x = (extra->x + sample.x) * m_cameraInvResolution.x;
            samplePos.y = (extra->y + sample.y) * m_cameraInvResolution.y;
        }

        pRec.uv = Point2(samplePos.x * m_cameraResolution.x,
                         samplePos.y * m_cameraResolution.y);

        /* Compute the corresponding position on the
           near plane (in local camera space) */
        Point nearP = m_sampleToCamera(samplePos);

        /* Turn that into a normalized ray direction */
        Vector d = normalize(Vector(nearP));

        dRec.d = trafo(d);
        dRec.measure = ESolidAngle;
        dRec.pdf = m_cameraNormalization / (d.z * d.z * d.z);

        return Spectrum(1.0f);

    }

    Float pdfProjectorDirection(const DirectionSamplingRecord &dRec,
                                const PositionSamplingRecord &pRec) const {
        if (dRec.measure != ESolidAngle)
            return 0.0f;

        const Transform &trafo = m_projectorWorldTransform->eval(pRec.time);

        Vector d = trafo.inverse()(dRec.d);
        Float cosTheta = Frame::cosTheta(d);

        /* Check if the direction points behind the camera */
        if (cosTheta <= 0)
            return 0.0f;

        /* Compute the position on the plane at distance 1 */
        Float invCosTheta = 1.0f / cosTheta;
        Point2 p(d.x * invCosTheta, d.y * invCosTheta);


        /* Check if the point lies inside the chosen crop rectangle */
        if (!m_projectorImageRect->contains(p)) {
            return 0.0f;
        }

        Point samplePoint(p.x,p.y,1.0f);
        Point sample = m_projectorToSample(samplePoint);
        Float xPos = sample.x * m_projectorSize.x, yPos = sample.y * m_projectorSize.y;

        if(((yPos - pRec.cameraUV.y - m_rowTransform)<1.f &&
            (yPos - pRec.cameraUV.y - m_rowTransform)>=0.f)){
            return importance(d, EPROJ) * (Float) m_projectorResolution.y;
        }
        return  0.0f;
    }

    Float pdfCameraDirection(const DirectionSamplingRecord &dRec,
                             const PositionSamplingRecord &pRec) const {
        if (dRec.measure != ESolidAngle)
            return 0.0f;

        const Transform &trafo = m_cameraWorldTransform->eval(pRec.time);

        Vector d = trafo.inverse()(dRec.d);
        Float cosTheta = Frame::cosTheta(d);

        /* Check if the direction points behind the camera */
        if (cosTheta <= 0) {
            return 0.0f;
        }

        /* Compute the position on the plane at distance 1 */
        Float invCosTheta = 1.0f / cosTheta;
        Point2 p(d.x * invCosTheta, d.y * invCosTheta);


        /* Check if the point lies inside the chosen crop rectangle */
        if (!m_cameraImageRect->contains(p)) {
            return 0.0f;
        }

        Point samplePoint(p.x,p.y,1.0f);
        Point sample = m_cameraToSample(samplePoint);
        Float xPos = sample.x * m_cameraResolution.x, yPos = sample.y * m_cameraResolution.y;

        if((yPos - floor(pRec.cameraUV.y) > 1.f || yPos < pRec.cameraUV.y)) {
            return 0.f;
        }


        return importance(trafo.inverse()(dRec.d), ECAM);

    }

    Spectrum evalProjectorDirection(const DirectionSamplingRecord &dRec,
                                    const PositionSamplingRecord &pRec) const {
        if (dRec.measure != ESolidAngle)
            return Spectrum(0.0f);

        const Transform &trafo = m_projectorWorldTransform->eval(pRec.time);

        Vector d = trafo.inverse()(dRec.d);
        Float cosTheta = Frame::cosTheta(d);

        /* Check if the direction points behind the camera */
        if (cosTheta <= 0) {
            return Spectrum(0.0f);
        }

        /* Compute the position on the plane at distance 1 */
        Float invCosTheta = 1.0f / cosTheta;
        Point2 p(d.x * invCosTheta, d.y * invCosTheta);

        /* Check if the point lies inside the chosen crop rectangle */
        if (!m_projectorImageRect->contains(p)) {
            return Spectrum(0.0f);
        }

        Point samplePoint(p.x,p.y,1.0f);
        Point sample = m_projectorToSample(samplePoint);
        Float u = sample.x * m_projectorSize.x, v = sample.y * m_projectorSize.y;
        int xPos = math::floorToInt(u), yPos = math::floorToInt(v);

        if(((yPos - pRec.cameraUV.y - m_rowTransform)<1.f && (yPos - pRec.cameraUV.y - m_rowTransform)>=0.f)){
            return importance(d, EPROJ) * Spectrum(m_scale);
        }

        return Spectrum(0.f);

    }

    Spectrum evalCameraDirection(const DirectionSamplingRecord &dRec,
                                 const PositionSamplingRecord &pRec) const {
        if (dRec.measure != ESolidAngle)
            return Spectrum(0.0f);

        const Transform &trafo = m_cameraWorldTransform->eval(pRec.time);

        Vector d = trafo.inverse()(dRec.d);
        Float cosTheta = Frame::cosTheta(d);

        /* Check if the direction points behind the camera */
        if (cosTheta <= 0) {
            return Spectrum(0.0f);
        }

        /* Compute the position on the plane at distance 1 */
        Float invCosTheta = 1.0f / cosTheta;
        Point2 p(d.x * invCosTheta, d.y * invCosTheta);


        /* Check if the point lies inside the chosen crop rectangle */
        if (!m_cameraImageRect->contains(p)) {
            return Spectrum(0.0f);
        }

        Point samplePoint(p.x,p.y,1.0f);
        Point sample = m_cameraToSample(samplePoint);
        Float xPos = sample.x * m_cameraResolution.x, yPos = sample.y * m_cameraResolution.y;

        if(yPos - floor(pRec.cameraUV.y) > 1.f || yPos < floor(pRec.cameraUV.y)) {
            return Spectrum(0.f);
        }

        return Spectrum(importance(trafo.inverse()(dRec.d), ECAM));
    }

    Spectrum sampleProjectorDirect(DirectSamplingRecord &dRec, const Point2 &sample) const {
        const Transform &trafo = m_projectorWorldTransform->eval(dRec.time);

        /* Transform the reference point into the local coordinate system */
        Point refP = trafo.inverse().transformAffine(dRec.ref);

        /* Check if it is outside of the clip range */
        if (refP.z < m_projectorNearClip || refP.z > m_projectorFarClip) {
            dRec.pdf = 0.0f;
            return Spectrum(0.0f);
        }

        Point screenSample = m_projectorToSample(refP);
        dRec.uv = Point2(screenSample.x, screenSample.y);
        if (dRec.uv.x < 0 || dRec.uv.x > 1 ||
            dRec.uv.y < 0 || dRec.uv.y > 1) {
            dRec.pdf = 0.0f;
            return Spectrum(0.0f);
        }

        dRec.uv.x *= m_projectorResolution.x;
        dRec.uv.y *= m_projectorResolution.y;

        int32_t targetPos = math::floorToInt(dRec.cameraUV.y + m_rowTransform) % m_projectorSize.y;
        if(dRec.uv.y < targetPos || dRec.uv.y -  (Float)targetPos > 1.0f) {
            dRec.pdf = 0;
            return Spectrum(0.0f);
        }

        Vector localD(refP);
        Float dist = localD.length(),
                invDist = 1.0f / dist;
        localD *= invDist;

        dRec.p = trafo.transformAffine(Point(0.0f));
        dRec.d = (dRec.p - dRec.ref) * invDist;
        dRec.dist = dist;
        dRec.n = trafo(Vector(0.0f, 0.0f, 1.0f));
        dRec.pdf = 1;
        dRec.measure = EDiscrete;

        return Spectrum(importance(localD, EPROJ) * invDist * invDist * m_scale);

    }

    Spectrum sampleCameraDirect(DirectSamplingRecord &dRec, const Point2 &sample) const {
        const Transform &trafo = m_cameraWorldTransform->eval(dRec.time);

        /* Transform the reference point into the local coordinate system */
        Point refP = trafo.inverse().transformAffine(dRec.ref);

        /* Check if it is outside of the clip range */
        if (refP.z < m_cameraNearClip || refP.z > m_cameraFarClip) {
            dRec.pdf = 0.0f;
            return Spectrum(0.0f);
        }

        Point screenSample = m_cameraToSample(refP);
        dRec.uv = Point2(screenSample.x, screenSample.y);
        if (dRec.uv.x < 0 || dRec.uv.x > 1 ||
            dRec.uv.y < 0 || dRec.uv.y > 1) {
            dRec.pdf = 0.0f;
            return Spectrum(0.0f);
        }

        dRec.uv.x *= m_cameraResolution.x;
        dRec.uv.y *= m_cameraResolution.y;

        if(dRec.uv.y - floor(dRec.cameraUV.y) > 1.f ||dRec.uv.y < floor(dRec.cameraUV.y)){
            dRec.pdf = 0;
            return Spectrum(0.0f);
        }

        Vector localD(refP);
        Float dist = localD.length(),
                invDist = 1.0f / dist;
        localD *= invDist;


        dRec.p = trafo.transformAffine(Point(0.0f));
        dRec.d = (dRec.p - dRec.ref) * invDist;
        dRec.dist = dist;
        dRec.n = trafo(Vector(0.0f, 0.0f, 1.0f));
        dRec.pdf = 1;
        dRec.measure = EDiscrete;


        return Spectrum(
                importance(localD, ECAM) * invDist * invDist);

    }

    Float pdfProjectorDirect(const DirectSamplingRecord &dRec) const {
        return (dRec.measure == EDiscrete) ? 1.0f : 0.0f;
    }

    Float pdfCameraDirect(const DirectSamplingRecord &dRec) const {
        return (dRec.measure == EDiscrete) ? 1.0f : 0.0f;
    }


    std::string toString() const {
        std::ostringstream oss;
        oss << "rowProbing[" << endl
            << "  disparity = x :" << m_rowTransform << endl
            << "  y :" << m_colTransform << endl
            << "]";
        return oss.str();
    }


    MTS_DECLARE_CLASS()
};

MTS_IMPLEMENT_CLASS_S(RowProbing, false, Probe)
MTS_EXPORT_PLUGIN(RowProbing, "Row Probing Pattern");

MTS_NAMESPACE_END

