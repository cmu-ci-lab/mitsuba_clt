//
// Created by Jiatian Sun on 5/15/18.
//

#include <mitsuba/render/scene.h>
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
#include <mitsuba/bidir/probe.h>

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

    class PerspectiveEmitterImpl : public PerspectiveEmitter{
    public:
        typedef TSpectrum<half, SPECTRUM_SAMPLES> SpectrumHalf;
        typedef TMIPMap<Spectrum, SpectrumHalf> MIPMap;
        PerspectiveEmitterImpl(const Properties &props) : PerspectiveEmitter(props) {
            m_type |=  EDeltaPosition | EPerspectiveEmitter | EOnSurface | EDirectionSampleMapsToPixels;

            m_scale = props.getFloat("scale", 1.0f);
            m_rowTransform = props.getInteger("rowTransform", 0);
            m_colTransform = props.getInteger("colTransform", 0);
            if (props.getTransform("toWorld", Transform()).hasScale())
                Log(EError, "Scale factors in the emitter-to-world "
                        "transformation are not allowed!");
        }

        PerspectiveEmitterImpl(Stream *stream, InstanceManager *manager)
                : PerspectiveEmitter(stream, manager) {
            m_scale = stream->readFloat();
            m_rowTransform = stream->readInt();
            m_colTransform = stream->readInt();
            configure();
        }

        void serialize(Stream *stream, InstanceManager *manager) const {
            PerspectiveEmitter::serialize(stream, manager);
            stream->writeFloat(m_scale);
            stream->writeInt(m_rowTransform);
            stream->writeInt(m_colTransform);
        }

        void configure() {
            PerspectiveEmitter::configure();
            /**
             * These do the following (in reverse order):
             *
             * 1. Create transform from camera space to [-1,1]x[-1,1]x[0,1] clip
             *    coordinates (not taking account of the aspect ratio yet)
             *
             * 2+3. Translate and scale to shift the clip coordinates into the
             *    range from zero to one, and take the aspect ratio into account.
             */
            m_cameraToSample =
                    Transform::scale(Vector(-0.5f, -0.5f*m_aspect, 1.0f))
                    * Transform::translate(Vector(-1.0f, -1.0f/m_aspect, 0.0f))
                    * Transform::perspective(m_xfov, m_nearClip, m_farClip);

            m_sampleToCamera = m_cameraToSample.inverse();

            /* Precompute some data for importance(). Please
               look at that function for further details */
            Point min(m_sampleToCamera(Point(0, 0, 0))),
                    max(m_sampleToCamera(Point(1, 1, 0)));

            m_imageRect.reset();
            m_imageRect.expandBy(Point2(min.x, min.y) / min.z);
            m_imageRect.expandBy(Point2(max.x, max.y) / max.z);
            m_normalization = 1.0f / m_imageRect.getVolume();

        }

        /// Helper function that samples a direction from the environment map
        Point2 internalSampleDirection(Point2 sample, Spectrum &value, Float &pdf) const {
            /* Sample a discrete pixel position */
            uint32_t row = sampleReuse(m_cdfRows, m_size.y, sample.y),
                    col = sampleReuse(m_cdfCols + row * (m_size.x+1), m_size.x, sample.x);


            /* Using the remaining bits of precision to get the exact position of the sample */
            Point2 pos = Point2((Float) col, (Float) row) + sample;

            /* Bilinearly interpolate colors from the adjacent four neighbors */
            int xPos = math::floorToInt(pos.x), yPos = math::floorToInt(pos.y);

            value = m_mipmap->evalTexel(0, xPos, yPos);
            pdf = value.getLuminance() * m_rowWeights[math::clamp(yPos, 0, m_size.y-1)]  * m_normalSpectrum;

            pos.x = pos.x/m_size.x; pos.y = pos.y/m_size.y;
            return pos;
        }

        Spectrum internalEvalDirection(const Vector &d) const{
            Float cosTheta = Frame::cosTheta(d);

            /* Compute the position on the plane at distance 1 */
            Float invCosTheta = 1.0f / cosTheta;
            Point2 uv(d.x * invCosTheta, d.y * invCosTheta);

            /* Check if the point lies inside the chosen crop rectangle */
            if (!m_imageRect.contains(uv)) {
                return Spectrum(0.0f);
            }

            Point samplePoint(uv.x,uv.y,1.0f);
            Point sample = m_cameraToSample(samplePoint * 1.0f);

            /* Convert to fractional pixel coordinates on the specified level */
            Float u = sample.x * m_size.x, v = sample.y * m_size.y;

            /* Take color from the pixel that sample resides */
            int xPos = math::floorToInt(u), yPos = math::floorToInt(v);

            Spectrum value = m_mipmap->evalTexel(0, xPos, yPos);
            return  value.getLuminance() * m_rowWeights[math::clamp(yPos,   0, m_size.y-1)] * m_normalSpectrum
                    * m_normalization * invCosTheta * invCosTheta * invCosTheta * value;

        }

        /// Helper function that computes the solid angle density of \ref internalSampleDirection()
        Float internalPdfDirection(const Vector &d) const {
            Float cosTheta = Frame::cosTheta(d);

            /* Check if the direction points behind the camera */
            if (cosTheta <= 0)
                return 0.0f;

            /* Compute the position on the plane at distance 1 */
            Float invCosTheta = 1.0f / cosTheta;
            Point2 uv(d.x * invCosTheta, d.y * invCosTheta);


            /* Check if the point lies inside the chosen crop rectangle */
            if (!m_imageRect.contains(uv)) {
                return 0.0f;
            }

            Point samplePoint(uv.x,uv.y,1.0f);
            Point sample = m_cameraToSample(samplePoint * m_nearClip);

            /* Take color from the pixel that sample resides */
            Float u = sample.x * m_size.x, v = sample.y * m_size.y;
            int xPos = math::floorToInt(u), yPos = math::floorToInt(v);
            Float dx1 = u - xPos, dx2 = 1.0f - dx1,
                    dy1 = v - yPos, dy2 = 1.0f - dy1;

            Spectrum value = m_mipmap->evalTexel(0,xPos,yPos);

            return value.getLuminance() * m_rowWeights[math::clamp(yPos,   0, m_size.y-1)]
                   * m_normalSpectrum * m_normalization * invCosTheta * invCosTheta * invCosTheta;
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
        inline Float importance(const Vector &d) const {
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
            if (!m_imageRect.contains(p))
                return 0.0f;

            return m_normalization * invCosTheta
                   * invCosTheta * invCosTheta;
        }

        Transform getProjectionTransform(const Point2 &apertureSample,
                                         const Point2 &aaSample) const {
            Float right = std::tan(m_xfov * M_PI/360) * m_nearClip, left = -right;
            Float top = right / m_aspect, bottom = -top;

            Vector2 offset(
                    (right-left)/10 * (aaSample.x-0.5f),
                    (top-bottom)/10* (aaSample.y-0.5f));

            return Transform::glFrustum(left+offset.x, right+offset.x,
                                        bottom+offset.y, top+offset.y, m_nearClip, m_farClip);
        }
        Spectrum samplePosition(PositionSamplingRecord &pRec,
                                const Point2 &sample, const Point2 *extra) const {
            const Transform &trafo = m_worldTransform->eval(pRec.time);
            pRec.p = trafo(Point(0.0f));
            pRec.n = trafo(Vector(0.0f, 0.0f, 1.0f));
            pRec.pdf = 1.0f;
            pRec.measure = EDiscrete;

            return Spectrum(1.0f);
        }

        Spectrum evalPosition(const PositionSamplingRecord &pRec) const {
            return Spectrum((pRec.measure == EDiscrete) ? 1.0f : 0.0f);
        }

        Float pdfPosition(const PositionSamplingRecord &pRec) const {
            return (pRec.measure == EDiscrete) ? 1.0f  : 0.0f;
        }

        Spectrum sampleDirection(DirectionSamplingRecord &dRec,
                                 PositionSamplingRecord &pRec,
                                 const Point2 &sample, const Point2 *extra) const {
            const Transform &trafo = m_worldTransform->eval(pRec.time);

            Point samplePos(sample.x, sample.y, 0.0f);
            if (extra) {
                /* The caller wants to condition on a specific pixel position */
                samplePos.x = (extra->x + sample.x) * m_invResolution.x;
                samplePos.y = (extra->y + sample.y) * m_invResolution.y;
            }

          if(pRec.probeType != Probe::ENORMAL){
            Point2 uvSample(samplePos.x * m_resolution.x, samplePos.y * m_resolution.y);
            int32_t xUnitTransform = m_colTransform % m_size.x;
            int32_t  xFinalTransform = xUnitTransform < 0 ? xUnitTransform + m_size.x: xUnitTransform;
            int32_t yUnitTransform = m_rowTransform % m_size.y;
            int32_t  yFinalTransform = yUnitTransform < 0 ? yUnitTransform + m_size.y: yUnitTransform;

            int32_t xPixelPos = (int32_t)floor(uvSample.x + xFinalTransform)% m_size.x;
            int32_t yPixelPos = (int32_t)floor(uvSample.y + yFinalTransform) % m_size.y;
            Float samplePosRelativeToPixelx = uvSample.x-(int32_t)floor(uvSample.x);
            Float samplePosRelativeToPixely = uvSample.y-(int32_t)floor(uvSample.y);

            samplePos = Point((xPixelPos + samplePosRelativeToPixelx) * m_invResolution.x,
                              (yPixelPos + samplePosRelativeToPixely) * m_invResolution.y,
                              0.0f);

            pRec.uv = Point2(samplePos.x * m_resolution.x,
                             samplePos.y * m_resolution.y);

            /* Compute the corresponding position on the
               near plane (in local camera space) */
            Point nearP = m_sampleToCamera(samplePos);

            /* Turn that into a normalized ray direction */
            Vector d = normalize(Vector(nearP));
            dRec.d = trafo(d);
            dRec.measure = ESolidAngle;
            if(pRec.probeType == Probe::ECOLUMN) {
              dRec.pdf = (Float) m_resolution.x * m_normalization / (d.z * d.z * d.z);
              return Spectrum(m_scale) / (Float) m_resolution.x;
            }
            else if(pRec.probeType == Probe::EROW){
              dRec.pdf = (Float)m_resolution.y * m_normalization / (d.z * d.z * d.z);
              return Spectrum(m_scale)/ (Float)m_resolution.y;
            }
            else if(pRec.probeType == Probe::EIDENTITY){
              dRec.pdf = m_normalization / (d.z * d.z * d.z) * m_resolution.x * m_resolution.y;
              return Spectrum(m_scale)/((Float)m_resolution.x * (Float)m_resolution.y);
            }
            else{
              dRec.pdf = m_normalization / (d.z * d.z * d.z);
              return Spectrum(m_scale);
            }
          }
          else {

            /* Sample a direction from the texture map */
            Spectrum value;
            Float pdf;
            Point2 pos = internalSampleDirection(Point2(samplePos.x, samplePos.y), value, pdf);

            samplePos.x = pos.x;
            samplePos.y = pos.y;
            Point2 uv(samplePos.x, samplePos.y);

            pRec.uv = Point2(samplePos.x * m_resolution.x, samplePos.y * m_resolution.y);

            /* Compute the corresponding position on the
               near plane (in local emitter space) */
            Point nearP = m_sampleToCamera(samplePos);

            /* Turn that into a normalized ray direction */
            Vector d = normalize(Vector(nearP));

            dRec.d = trafo(d);
            dRec.measure = ESolidAngle;
            dRec.pdf = m_normalization / (d.z * d.z * d.z) * pdf;

            if (value.isZero() || pdf == 0)
              return Spectrum(0.0f);
            else {
              return value * m_scale / (m_normalSpectrum * m_resolution.x * m_resolution.y);

            }
          }
        }

        Float pdfDirection(const DirectionSamplingRecord &dRec,
                           const PositionSamplingRecord &pRec) const {
            if (dRec.measure != ESolidAngle)
                return 0.0f;

            const Transform &trafo = m_worldTransform->eval(pRec.time);

            uint32_t probeType = pRec.probeType;
            if(probeType == Probe::EROW)
              return importance(trafo.inverse()(dRec.d)) * (Float)m_resolution.y;
            else if(probeType == Probe::EIDENTITY)
              return importance(trafo.inverse()(dRec.d)) * ((Float)m_resolution.x * (Float)m_resolution.y);
            else if(probeType == Probe::ECOLUMN)
              return importance(trafo.inverse()(dRec.d)) * (Float)m_resolution.x;
            else
              return internalPdfDirection(trafo.inverse()(dRec.d));
        }

        Spectrum evalDirection(const DirectionSamplingRecord &dRec,
                               const PositionSamplingRecord &pRec) const {
            if (dRec.measure != ESolidAngle)
                return Spectrum(0.0f);

            const Transform &trafo = m_worldTransform->eval(pRec.time);

            Vector v = trafo.inverse()(dRec.d);
            uint32_t probeType = pRec.probeType;

            if(probeType != Probe::ENORMAL)
              return importance(v) * Spectrum(m_scale);
            else
              return internalEvalDirection(v) * m_scale /(m_normalSpectrum * m_resolution.x * m_resolution.y);

        }

        Spectrum sampleRay(Ray &ray, const Point2 &pixelSample,
                           const Point2 &otherSample, Float timeSample) const {
            ray.setTime(timeSample);

            /* Compute the corresponding position on the
               near plane (in local emitter space) */
            Point nearP = m_sampleToCamera(Point(
                    pixelSample.x * m_invResolution.x,
                    pixelSample.y * m_invResolution.y, 0.0f));

            Point2 uv(pixelSample.x,pixelSample.y);

            /* Turn that into a normalized ray direction, and
               adjust the ray interval accordingly */
            Vector d = normalize(Vector(nearP));
            Float invZ = 1.0f / d.z;
            ray.mint = m_nearClip * invZ;
            ray.maxt = m_farClip * invZ;

            const Transform &trafo = m_worldTransform->eval(ray.time);
            ray.setOrigin(trafo.transformAffine(Point(0.0f)));
            ray.setDirection(trafo(d));

            return Spectrum(m_scale * m_mipmap->evalTexel(0,math::floorToInt(uv.x * m_resolution.x),
                                                          math::floorToInt(uv.y * m_resolution.y)));
        }

        Spectrum sampleDirect(DirectSamplingRecord &dRec, const Point2 &sample) const {
            const Transform &trafo = m_worldTransform->eval(dRec.time);

            /* Transform the reference point into the local coordinate system */
            Point refP = trafo.inverse().transformAffine(dRec.ref);

            /* Check if it is outside of the clip range */
            if (refP.z < m_nearClip || refP.z > m_farClip) {
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
            //extra point added by myself
            Point2 uv(dRec.uv.x, dRec.uv.y);
            dRec.uv.x *= m_resolution.x;
            dRec.uv.y *= m_resolution.y;

          if((dRec.probeType==Probe::ECOLUMN) && floor(dRec.uv.x) != floor(dRec.pixelPosition.x)) {
            dRec.pdf = 0;
            return Spectrum(0.0f);
          }
          else if((dRec.probeType==Probe::EROW) && floor(dRec.uv.y) != floor(dRec.pixelPosition.y)) {
            dRec.pdf = 0;
            return Spectrum(0.0f);
          }
          else if((dRec.probeType==Probe::EIDENTITY) && (floor(dRec.uv.x) != floor(dRec.pixelPosition.x)
                                                         || floor(dRec.uv.y) != floor(dRec.pixelPosition.y))) {
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

          int32_t probeType = dRec.probeType;

          if(probeType != Probe::ENORMAL)
            return Spectrum(importance(localD) * invDist * invDist * m_scale);
          else
            return importance(localD) * invDist * invDist * m_scale *
                   m_mipmap->evalTexel(0,math::floorToInt(dRec.uv.x),math::floorToInt(dRec.uv.y));

        }

        Float pdfDirect(const DirectSamplingRecord &dRec) const {
            return (dRec.measure == EDiscrete) ? 1.0f : 0.0f;
        }

        AABB getAABB() const {
            return AABB();
        }


        std::string toString() const {
            std::ostringstream oss;
            oss << "PerspectiveEmitter[" << endl
                << "  scale = " << m_scale << "," << endl
                << "  samplingWeight = " << m_samplingWeight << "," << endl
                << "  worldTransform = " << indent(m_worldTransform.toString()) << "," << endl
                << "  medium = " << indent(m_medium.toString()) << endl
                << "]";
            return oss.str();
        }



        MTS_DECLARE_CLASS()
    private:
        /// Sample from an array using the inversion method
        inline uint32_t sampleReuse(float *cdf, uint32_t size, Float &sample) const {
            float *entry = std::lower_bound(cdf, cdf+size+1, (float) sample);
            uint32_t index = std::min((uint32_t) std::max((ptrdiff_t) 0, entry - cdf - 1), size-1);
            sample = (sample - (Float) cdf[index]) / (Float) (cdf[index+1] - cdf[index]);
            return index;
        }
    private:
        Float m_scale;
        Float m_invSurfaceArea;
        Transform m_cameraToSample;
        Transform m_sampleToCamera;
        AABB2 m_imageRect;
        Float m_normalization;

    };

    MTS_IMPLEMENT_CLASS_S(PerspectiveEmitterImpl, false, PerspectiveEmitter)
    MTS_EXPORT_PLUGIN(PerspectiveEmitterImpl, "Perspective emitter");

MTS_NAMESPACE_END