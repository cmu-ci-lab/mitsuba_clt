//
// Created by Jiatian Sun on 2019-07-08.
//
#include <mitsuba/render/probe.h>
//
// Created by Jiatian Sun on 9/13/18.
//

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
#include <mitsuba/core/imagePlane_boundary.h>
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

class EpipolarProbing : public BasicProbe{
public:
    typedef TSpectrum<half, SPECTRUM_SAMPLES> SpectrumHalf;
    typedef TMIPMap<Spectrum, SpectrumHalf> MIPMap;
    EpipolarProbing(const Properties &props) : BasicProbe(props) {
        m_rowTransform = props.getInteger("rowTransform", 0);
        m_colTransform = props.getInteger("colTransform", 0);
        m_type = "epipolar";
    }

    EpipolarProbing(Stream *stream, InstanceManager *manager)
            : BasicProbe(stream, manager) {
        m_rowTransform = stream->readInt();
        m_colTransform = stream->readInt();
    }

    void serialize(Stream *stream, InstanceManager *manager) const {
        Probe::serialize(stream, manager);
        stream->writeInt(m_rowTransform);
        stream->writeInt(m_colTransform);
    }

    void readFromProjector(Stream *stream) {
        BasicProbe::readFromProjector(stream);
    }

    void readFromCamera(Stream *stream) {
        BasicProbe::readFromCamera(stream);
        const Transform &trafo = m_cameraWorldTransform->eval(0.0);
        m_worldCameraOrigin = trafo(Point(0.0f));
        m_worldCameraZAxis = trafo(Point(0.f,0.f,1.f));
        m_worldCameraYAxis = trafo(Point(0.f,1.0f,0.0f));
        m_worldCameraXAxis = trafo(Point(1.f,0.0f,0.0f));
    }

    void configureAfterLoadingCamProj(){
        BasicProbe::configureAfterLoadingCamProj();
        const Transform &trafo = m_projectorWorldTransform->eval(0);
        const Transform &camTrafo = m_cameraWorldTransform->eval(0);

        //Find the y, z direction axis of the projector
        Point currentCenter = trafo(Point(0.0f));
        Vector emitterZDirection = normalize(trafo(Point(0.f, 0.f, 1.f)) - currentCenter);
        Vector emitterYDirection = normalize(trafo(Point(0.f, 1.0f, 0.f)) - currentCenter);
        Vector emitterXDirection = normalize(trafo(Point(1.f, 0.0f, 0.f)) - currentCenter);

        //Change the projector center into camera's local coordinate
        Point translaP = camTrafo.inverse()(currentCenter);
        Vector translation(translaP.x, translaP.y, translaP.z);
        Vector sensorZDirection = normalize(m_worldCameraZAxis - m_worldCameraOrigin);
        Vector sensorYDirection = normalize(m_worldCameraYAxis - m_worldCameraOrigin);
        Vector sensorXDirection = normalize(m_worldCameraXAxis - m_worldCameraOrigin);

        //Calculate rotation matrix that rotates emitter's orientation to
        //to sensor's orientation
        Vector emitterZDirectionInCamView = normalize(camTrafo.inverse()(emitterZDirection));
        Vector emitterYDirectionInCamView = normalize(camTrafo.inverse()(emitterYDirection));
        Vector emitterXDirectionInCamView = normalize(camTrafo.inverse()(emitterXDirection));

        Matrix3x3 rotation( Vector( emitterXDirectionInCamView.x,  emitterYDirectionInCamView.x,  emitterZDirectionInCamView.x),
                       Vector( emitterXDirectionInCamView.y,  emitterYDirectionInCamView.y,  emitterZDirectionInCamView.y),
                       Vector( emitterXDirectionInCamView.z,  emitterYDirectionInCamView.z,  emitterZDirectionInCamView.z));

        Matrix3x3 crossMatrix;

        crossMatrix(0, 0) = 0;
        crossMatrix(0, 1) = -translation.z;
        crossMatrix(0, 2) = translation.y;

        crossMatrix(1, 0) = translation.z;
        crossMatrix(1, 1) = 0;
        crossMatrix(1, 2) = -translation.x;

        crossMatrix(2, 0) = -translation.y;
        crossMatrix(2, 1) = translation.x;
        crossMatrix(2, 2) = 0;

        funda = rotation * crossMatrix;
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
        const Transform &camTrafo = m_cameraWorldTransform->eval(pRec.time);

        Point epipolarLine = Point(funda * Vector(pRec.worldCameraSample));
        pRec.epiline = epipolarLine;

        Point camTOP = m_sampleToProjector(Point(0.f, 0.f, 0.f));
        Point camBOT = m_sampleToProjector(Point(1.f, 1.f, 0.f));

        Vector dirTOP = normalize(Vector(camTOP.x, camTOP.y, camTOP.z));
        Vector dirBOT = normalize(Vector(camBOT.x, camBOT.y, camBOT.z));
        Float cosThetaTOP = Frame::cosTheta(dirTOP);
        Float cosThetaBOT = Frame::cosTheta(dirBOT);

        Point normCamTOP(0.f, 0.f, 1.f);
        Point normCamBOT(0.f, 0.f, 1.f);
        /* Check if the direction points behind the camera */
        if (cosThetaTOP > 0) { // assume this condition is always true
            /* Compute the position on the plane at distance 1 */
            Float invCosThetaTOP = 1.0f / cosThetaTOP;
            normCamTOP.x = dirTOP.x * invCosThetaTOP;
            normCamTOP.y = dirTOP.y * invCosThetaTOP;
        }

        if (cosThetaTOP > 0) { // assume this condition is always true
            /* Compute the position on the plane at distance 1 */
            Float invCosThetaBOT = 1.0f / cosThetaBOT;
            normCamBOT.x = dirBOT.x * invCosThetaBOT;
            normCamBOT.y = dirBOT.y * invCosThetaBOT;
        }

        Float yMIN = normCamTOP.y < normCamBOT.y ? normCamTOP.y : normCamBOT.y;
        Float yMAX = normCamTOP.y < normCamBOT.y ? normCamBOT.y : normCamTOP.y;
        Float xMIN = normCamTOP.x < normCamBOT.x ? normCamTOP.x : normCamBOT.x;
        Float xMAX = normCamTOP.x < normCamBOT.x ? normCamBOT.x : normCamTOP.x;
        Float planeWidth = xMAX - xMIN;
        Float planeHeight = yMAX - yMIN;

        Float pixelWidth = (xMAX - xMIN) * m_projectorInvResolution.x;
        Float halfPixelW = pixelWidth / 2.f;

        Float camIntvAngle;
        Point upperLine, lowerLine;
        if (fabs(epipolarLine.y) >= 1e-9) {
            float k = -epipolarLine.x / epipolarLine.y;
            camIntvAngle = atan(k);
            if (fabs(camIntvAngle) < 1e-6) {
                //Horizontal lines
                upperLine = Point(epipolarLine.x, epipolarLine.y, epipolarLine.z + epipolarLine.y * halfPixelW);
                lowerLine = Point(epipolarLine.x, epipolarLine.y, epipolarLine.z - epipolarLine.y * halfPixelW);
            } else {
                float ascendScale = fabs(halfPixelW * epipolarLine.y / cos(camIntvAngle));
                upperLine = Point(epipolarLine.x, epipolarLine.y, epipolarLine.z + ascendScale);
                lowerLine = Point(epipolarLine.x, epipolarLine.y, epipolarLine.z - ascendScale);
            }
        } else {
            //Vertical lines
            upperLine = Point(epipolarLine.x, epipolarLine.y, epipolarLine.z + epipolarLine.x * halfPixelW);
            lowerLine = Point(epipolarLine.x, epipolarLine.y, epipolarLine.z - epipolarLine.x * halfPixelW);

        }


        LineEnds upperLineEnds, lowerLineEnds, middleLineEnds;
        getLineEnds(upperLineEnds, dirTOP, dirBOT, upperLine);
        getLineEnds(lowerLineEnds, dirTOP, dirBOT, lowerLine);
        getLineEnds(middleLineEnds, dirTOP, dirBOT, epipolarLine);


        float randNumber = sampler->next1D();

        Point2 reCamSampled;
        if (upperLineEnds.isInvalid && lowerLineEnds.isInvalid && middleLineEnds.isInvalid) { // case 1
            pRec.epiLineLen = -1;
            return Spectrum(0.f);
        } else if (upperLineEnds.isInvalid && lowerLineEnds.isInvalid && !middleLineEnds.isInvalid) { // case 2
            Point camSample = m_sampleToProjector(Point(sample.x, sample.y, 0.f));
            Vector sampleDirection = normalize(Vector(camSample.x, camSample.y, camSample.z));
            Float cosSampleTheta = Frame::cosTheta(sampleDirection);
            /* Check if the direction points behind the camera */
            if (cosSampleTheta <= 0) {
                pRec.epiLineLen = -1;
                return Spectrum(0.f);
            }
            /* Compute the position on the plane at distance 1 */
            Float invCosSampleTheta = 1.0f / cosSampleTheta;
            reCamSampled.x = sampleDirection.x * invCosSampleTheta;
            reCamSampled.y = sampleDirection.y * invCosSampleTheta;
            pRec.epiLineLen = (yMAX - yMIN) * (xMAX - xMIN);
        } else if (!lowerLineEnds.isInvalid && lowerLineEnds.isAdjacent && upperLineEnds.isInvalid) {  // case 3
            Float toCornerDist = getPoint2LineDistance(lowerLineEnds.corner, lowerLine);
            if (lowerLineEnds.startType == Boundary::EUP || lowerLineEnds.endType == Boundary::EUP) {
                Point2 vertices[3] = {lowerLineEnds.start, lowerLineEnds.end, lowerLineEnds.corner};
                reCamSampled = samplePolygon(vertices, 3, randNumber, sample, pRec.epiLineLen);
            } else {
                if (lowerLineEnds.startType == Boundary::ELEFT) {
                    Point2 vertices[5] = {lowerLineEnds.start, Point2(xMIN, yMIN), Point2(xMAX, yMIN),
                                          Point2(xMAX, yMAX), lowerLineEnds.end};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (lowerLineEnds.endType == Boundary::ELEFT) {
                    Point2 vertices[5] = {lowerLineEnds.end, Point2(xMIN, yMIN), Point2(xMAX, yMIN),
                                          Point2(xMAX, yMAX), lowerLineEnds.start};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (lowerLineEnds.startType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {lowerLineEnds.start, Point2(xMAX, yMIN),
                                          Point2(xMIN, yMIN), Point2(xMIN, yMAX), lowerLineEnds.end};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (lowerLineEnds.endType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {lowerLineEnds.end, Point2(xMAX, yMIN),
                                          Point2(xMIN, yMIN), Point2(xMIN, yMAX), lowerLineEnds.start};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else {
                    SLog(EError, "unexpected condition for case 3");
                }
            }
        } else if (!lowerLineEnds.isInvalid && !lowerLineEnds.isAdjacent && upperLineEnds.isInvalid) { //case 4
//            if (lowerLineEnds.startType == lowerLineEnds.endType){
//                if((lowerLineEnds.end - lowerLineEnds.start).length() < 1e-6) {
//                    if(fabs(lowerLineEnds.end.y- yMIN)< 1e-6 || fabs(lowerLineEnds.start.y- yMIN)< 1e-6) {
//                        pRec.epiLineLen = -1;
//                        return Spectrum(0.f);
//                    }
//                    else if(fabs(lowerLineEnds.end.y- yMAX)< 1e-6 || fabs(lowerLineEnds.start.y- yMAX)< 1e-6) {
//                        Point camSample = m_sampleToProjector(Point(sample.x, sample.y, 0.f));
//                        Vector sampleDirection = normalize(Vector(camSample.x, camSample.y, camSample.z));
//                        Float cosSampleTheta = Frame::cosTheta(sampleDirection);
//                        /* Check if the direction points behind the camera */
//                        if (cosSampleTheta <= 0) {
//                            pRec.epiLineLen = -1;
//                            return Spectrum(0.f);
//                        }
//                        /* Compute the position on the plane at distance 1 */
//                        Float invCosSampleTheta = 1.0f / cosSampleTheta;
//                        reCamSampled.x = sampleDirection.x * invCosSampleTheta;
//                        reCamSampled.y = sampleDirection.y * invCosSampleTheta;
//                        pRec.epiLineLen = (yMAX - yMIN) * (xMAX - xMIN);
//                    }
//                    else{
//                        SLog(EError, "unexpected condition for case 4 when there is only one intersection point");
//                    }
//                }
//                else{ //assume this epiline is exactly on the edge of the boundery
//                    if ((fabs(lowerLineEnds.end.y - lowerLineEnds.start.y) < 1e-6 &&
//                         fabs(lowerLineEnds.end.y - yMAX) < 1e-6) ||
//                        (fabs(lowerLineEnds.end.x - lowerLineEnds.start.x) < 1e-5 &&
//                         fabs(lowerLineEnds.end.x - xMAX) < 1e-6)) {
//                        Point camSample = m_sampleToProjector(Point(sample.x, sample.y, 0.f));
//                        Vector sampleDirection = normalize(Vector(camSample.x, camSample.y, camSample.z));
//                        Float cosSampleTheta = Frame::cosTheta(sampleDirection);
//                        /* Check if the direction points behind the camera */
//                        if (cosSampleTheta <= 0) {
//                            pRec.epiLineLen = -1;
//                            return Spectrum(0.f);
//                        }
//                        /* Compute the position on the plane at distance 1 */
//                        Float invCosSampleTheta = 1.0f / cosSampleTheta;
//                        reCamSampled.x = sampleDirection.x * invCosSampleTheta;
//                        reCamSampled.y = sampleDirection.y * invCosSampleTheta;
//                        pRec.epiLineLen = (yMAX - yMIN) * (xMAX - xMIN);
//                    } else if((fabs(lowerLineEnds.end.y - lowerLineEnds.start.y) < 1e-6 &&
//                                 fabs(lowerLineEnds.end.y - yMIN) < 1e-6) ||
//                                (fabs(lowerLineEnds.end.x - lowerLineEnds.start.x) < 1e-6 &&
//                                 fabs(lowerLineEnds.end.x - xMIN) < 1e-6)){
//                        pRec.epiLineLen = -1;
//                        return Spectrum(0.f);
//
//                    }
//                    else{
//                        SLog(EError, "unexpected condition for case 4 when there is only one intersection edge");
//                    }
//                }
//            }
            if (lowerLineEnds.startType == Boundary::ELEFT) {
                Point2 vertices[4] = {lowerLineEnds.start, Point2(xMIN, yMIN),
                                      Point2(xMAX, yMIN), lowerLineEnds.end};
                reCamSampled = samplePolygon(vertices, 4, randNumber, sample, pRec.epiLineLen);
            } else if (lowerLineEnds.endType == Boundary::ELEFT) {
                Point2 vertices[4] = {lowerLineEnds.end, Point2(xMIN, yMIN),
                                      Point2(xMAX, yMIN), lowerLineEnds.start};
                reCamSampled = samplePolygon(vertices, 4, randNumber, sample, pRec.epiLineLen);
            } else if (lowerLineEnds.startType == Boundary::EUP) {
                Point2 vertices[4] = {lowerLineEnds.start, Point2(xMIN, yMIN),
                                      Point2(xMIN, yMAX), lowerLineEnds.end};
                reCamSampled = samplePolygon(vertices, 4, randNumber, sample, pRec.epiLineLen);
            } else if (lowerLineEnds.endType == Boundary::EUP) {
                Point2 vertices[4] = {lowerLineEnds.end, Point2(xMIN, yMIN),
                                      Point2(xMIN, yMAX), lowerLineEnds.start};
                reCamSampled = samplePolygon(vertices, 4, randNumber, sample, pRec.epiLineLen);
            }
            else {
                printf("lowerLineEnds endType %d startType %d endPoint %s startPoint %s %f x + %f y + %f = 0 \n",
                        lowerLineEnds.endType, lowerLineEnds.startType, lowerLineEnds.end.toString().c_str(),
                       lowerLineEnds.start.toString().c_str(),
                       lowerLine.x, lowerLine.y, lowerLine.z);
                SLog(EError, "unexpected condition for case 4");
            }
        } else if (!upperLineEnds.isInvalid && upperLineEnds.isAdjacent && lowerLineEnds.isInvalid) { //case 5
            Float toCornerDist = getPoint2LineDistance(upperLineEnds.corner, upperLine);
            if (upperLineEnds.startType == Boundary::EDOWN || upperLineEnds.endType == Boundary::EDOWN) {
                Point2 vertices[3] = {upperLineEnds.start, upperLineEnds.end, upperLineEnds.corner};
                reCamSampled = samplePolygon(vertices, 3, randNumber, sample, pRec.epiLineLen);
            } else {
                if (upperLineEnds.startType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.start, Point2(xMIN, yMAX), Point2(xMAX, yMAX),
                                          Point2(xMAX, yMIN), upperLineEnds.end};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.end, Point2(xMIN, yMAX), Point2(xMAX, yMAX),
                                          Point2(xMAX, yMIN), upperLineEnds.start};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.start, Point2(xMAX, yMAX),
                                          Point2(xMIN, yMAX), Point2(xMIN, yMIN), upperLineEnds.end};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.end, Point2(xMAX, yMAX),
                                          Point2(xMIN, yMAX), Point2(xMIN, yMIN), upperLineEnds.start};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else {
                    SLog(EError, "unexpected condition for case 5");
                }
            }
        } else if (!upperLineEnds.isInvalid && !upperLineEnds.isAdjacent && lowerLineEnds.isInvalid) { //case 6
//            if (upperLineEnds.startType == upperLineEnds.endType){
//                if((upperLineEnds.end - upperLineEnds.start).length() < 1e-6) {
//                    if(fabs(upperLineEnds.end.y- yMAX)< 1e-6 || fabs(upperLineEnds.start.y- yMAX)< 1e-6) {
//                        pRec.epiLineLen = -1;
//                        return Spectrum(0.f);
//                    }
//                    else if(fabs(upperLineEnds.end.y- yMIN)< 1e-6 || fabs(upperLineEnds.start.y- yMIN)< 1e-6) {
//                        Point camSample = m_sampleToProjector(Point(sample.x, sample.y, 0.f));
//                        Vector sampleDirection = normalize(Vector(camSample.x, camSample.y, camSample.z));
//                        Float cosSampleTheta = Frame::cosTheta(sampleDirection);
//                        /* Check if the direction points behind the camera */
//                        if (cosSampleTheta <= 0) {
//                            pRec.epiLineLen = -1;
//                            return Spectrum(0.f);
//                        }
//                        /* Compute the position on the plane at distance 1 */
//                        Float invCosSampleTheta = 1.0f / cosSampleTheta;
//                        reCamSampled.x = sampleDirection.x * invCosSampleTheta;
//                        reCamSampled.y = sampleDirection.y * invCosSampleTheta;
//                        pRec.epiLineLen = (yMAX - yMIN) * (xMAX - xMIN);
//                    }
//                    else{
//                        SLog(EError, "unexpected condition for case 6 when there is only one intersection point");
//                    }
//                }
//                else{ //assume this epiline is exactly on the edge of the boundery
//                    if ((fabs(upperLineEnds.end.y - upperLineEnds.start.y) < 1e-6 &&
//                         fabs(upperLineEnds.end.y - yMIN) < 1e-6) ||
//                        (fabs(upperLineEnds.end.x - upperLineEnds.start.x) < 1e-6 &&
//                         fabs(upperLineEnds.end.x - xMIN) < 1e-6)) {
//                        Point camSample = m_sampleToProjector(Point(sample.x, sample.y, 0.f));
//                        Vector sampleDirection = normalize(Vector(camSample.x, camSample.y, camSample.z));
//                        Float cosSampleTheta = Frame::cosTheta(sampleDirection);
//                        /* Check if the direction points behind the camera */
//                        if (cosSampleTheta <= 0) {
//                            pRec.epiLineLen = -1;
//                            return Spectrum(0.f);
//                        }
//                        /* Compute the position on the plane at distance 1 */
//                        Float invCosSampleTheta = 1.0f / cosSampleTheta;
//                        reCamSampled.x = sampleDirection.x * invCosSampleTheta;
//                        reCamSampled.y = sampleDirection.y * invCosSampleTheta;
//                        pRec.epiLineLen = (yMAX - yMIN) * (xMAX - xMIN);
//                    } else if((fabs(upperLineEnds.end.y - upperLineEnds.start.y) < 1e-6 &&
//                               fabs(upperLineEnds.end.y - yMAX) < 1e-6) ||
//                              (fabs(upperLineEnds.end.x - upperLineEnds.start.x) < 1e-6 &&
//                               fabs(upperLineEnds.end.x - xMAX) < 1e-6)){
//                        pRec.epiLineLen = -1;
//                        return Spectrum(0.f);
//
//                    }
//                    else{
//                        printf("xMin %f xMax %f yMin%f yMAX%f",xMIN, xMAX, yMIN, yMAX);
//                        printf("upperLineEnds endType %d startType %d endPoint %s startPoint %s %f x + %f y + %f = 0 \n",
//                               upperLineEnds.endType, upperLineEnds.startType, upperLineEnds.end.toString().c_str(),
//                               upperLineEnds.start.toString().c_str(),
//                               upperLine.x, upperLine.y, upperLine.z);
//                        SLog(EError, "unexpected condition for case 6 when there is only one intersection edge");
//                    }
//                }
//            }
            if (upperLineEnds.startType == Boundary::ELEFT) {
                Point2 vertices[4] = {upperLineEnds.start, Point2(xMIN, yMAX),
                                      Point2(xMAX, yMAX), upperLineEnds.end};
                reCamSampled = samplePolygon(vertices, 4, randNumber, sample, pRec.epiLineLen);
            } else if (upperLineEnds.endType == Boundary::ELEFT) {
                Point2 vertices[4] = {upperLineEnds.end, Point2(xMIN, yMAX),
                                      Point2(xMAX, yMAX), upperLineEnds.start};
                reCamSampled = samplePolygon(vertices, 4, randNumber, sample, pRec.epiLineLen);
            } else if (upperLineEnds.startType == Boundary::EUP) {
                Point2 vertices[4] = {upperLineEnds.start, Point2(xMAX, yMIN),
                                      Point2(xMAX, yMAX), upperLineEnds.end};
                reCamSampled = samplePolygon(vertices, 4, randNumber, sample, pRec.epiLineLen);
            } else if (upperLineEnds.endType == Boundary::EUP) {
                Point2 vertices[4] = {upperLineEnds.end, Point2(xMAX, yMIN),
                                      Point2(xMAX, yMAX), upperLineEnds.start};
                reCamSampled = samplePolygon(vertices, 4, randNumber, sample, pRec.epiLineLen);
            } else {
                printf("xMin %f xMax %f yMin%f yMAX%f",xMIN, xMAX, yMIN, yMAX);
                printf("upperLineEnds endType %d startType %d endPoint %s startPoint %s %f x + %f y + %f = 0 \n",
                       upperLineEnds.endType, upperLineEnds.startType, upperLineEnds.end.toString().c_str(),
                       upperLineEnds.start.toString().c_str(),
                       upperLine.x, upperLine.y, upperLine.z);
                SLog(EError, "unexpected condition for case 6");
            }
        } else if (!upperLineEnds.isInvalid && !lowerLineEnds.isInvalid
                   && upperLineEnds.isAdjacent && lowerLineEnds.isAdjacent) { // case 7
            if (LineEnds::hasSameType(upperLineEnds, lowerLineEnds)) {
                if (upperLineEnds.startType == lowerLineEnds.startType) {
                    Point2 vertices[4] = {lowerLineEnds.start, lowerLineEnds.end,
                                          upperLineEnds.end, upperLineEnds.start};
                    reCamSampled = samplePolygon(vertices, 4, randNumber, sample, pRec.epiLineLen);
                } else {
                    Point2 vertices[4] = {lowerLineEnds.start, lowerLineEnds.end,
                                          upperLineEnds.start, upperLineEnds.end};
                    reCamSampled = samplePolygon(vertices, 4, randNumber, sample, pRec.epiLineLen);
                }
            } else {
                if (upperLineEnds.startType == Boundary::ELEFT) {
                    if (upperLineEnds.startType == lowerLineEnds.startType) {
                        Point2 vertices[6] = {upperLineEnds.start, upperLineEnds.end,
                                              Point2(xMAX, yMIN),
                                              lowerLineEnds.start, lowerLineEnds.end,
                                              Point2(xMIN, yMAX)};
                        reCamSampled = samplePolygon(vertices, 6, randNumber, sample, pRec.epiLineLen);
                    } else {
                        Point2 vertices[6] = {upperLineEnds.start, upperLineEnds.end,
                                              Point2(xMAX, yMIN),
                                              lowerLineEnds.end, lowerLineEnds.start,
                                              Point2(xMIN, yMAX)};
                        reCamSampled = samplePolygon(vertices, 6, randNumber, sample, pRec.epiLineLen);
                    }
                } else if (upperLineEnds.startType == Boundary::ERIGHT) {
                    if (upperLineEnds.startType == lowerLineEnds.startType) {
                        Point2 vertices[6] = {upperLineEnds.start, upperLineEnds.end,
                                              Point2(xMIN, yMIN),
                                              lowerLineEnds.start, lowerLineEnds.end,
                                              Point2(xMAX, yMAX)};
                        reCamSampled = samplePolygon(vertices, 6, randNumber, sample, pRec.epiLineLen);
                    } else {
                        Point2 vertices[6] = {upperLineEnds.start, upperLineEnds.end,
                                              Point2(xMIN, yMIN),
                                              lowerLineEnds.end, lowerLineEnds.start,
                                              Point2(xMAX, yMAX)};
                        reCamSampled = samplePolygon(vertices, 6, randNumber, sample, pRec.epiLineLen);
                    }
                }
                if (upperLineEnds.endType == Boundary::ELEFT) {
                    if (upperLineEnds.endType == lowerLineEnds.endType) {
                        Point2 vertices[6] = {upperLineEnds.end, upperLineEnds.start,
                                              Point2(xMAX, yMIN),
                                              lowerLineEnds.end, lowerLineEnds.start,
                                              Point2(xMIN, yMAX)};
                        reCamSampled = samplePolygon(vertices, 6, randNumber, sample, pRec.epiLineLen);
                    } else {
                        Point2 vertices[6] = {upperLineEnds.end, upperLineEnds.start,
                                              Point2(xMAX, yMIN),
                                              lowerLineEnds.start, lowerLineEnds.end,
                                              Point2(xMIN, yMAX)};
                        reCamSampled = samplePolygon(vertices, 6, randNumber, sample, pRec.epiLineLen);
                    }
                } else if (upperLineEnds.endType == Boundary::ERIGHT) {
                    if (upperLineEnds.endType == lowerLineEnds.endType) {
                        Point2 vertices[6] = {upperLineEnds.end, upperLineEnds.start,
                                              Point2(xMIN, yMIN),
                                              lowerLineEnds.end, lowerLineEnds.start,
                                              Point2(xMAX, yMAX)};
                        reCamSampled = samplePolygon(vertices, 6, randNumber, sample, pRec.epiLineLen);
                    } else {
                        Point2 vertices[6] = {upperLineEnds.end, upperLineEnds.start,
                                              Point2(xMIN, yMIN),
                                              lowerLineEnds.start, lowerLineEnds.end,
                                              Point2(xMAX, yMAX)};
                        reCamSampled = samplePolygon(vertices, 6, randNumber, sample, pRec.epiLineLen);
                    }
                }
            }
        } else if (!upperLineEnds.isInvalid && !lowerLineEnds.isInvalid
                   && !upperLineEnds.isAdjacent &&
                   lowerLineEnds.isAdjacent) { // case 8                                            )) { // case 8
            if (upperLineEnds.startType == lowerLineEnds.startType) {
                if (upperLineEnds.startType == Boundary::EDOWN &&
                    lowerLineEnds.endType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMIN, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == lowerLineEnds.startType &&
                           upperLineEnds.startType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMIN, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == lowerLineEnds.startType &&
                           upperLineEnds.startType == Boundary::EDOWN &&
                           lowerLineEnds.endType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMAX, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == lowerLineEnds.startType &&
                           upperLineEnds.startType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMAX, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                }
            } else if (upperLineEnds.endType == lowerLineEnds.endType) {
                if (upperLineEnds.endType == Boundary::EDOWN &&
                    lowerLineEnds.startType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMIN, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMIN, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::EDOWN &&
                           lowerLineEnds.startType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMAX, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMAX, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                }
            } else if (upperLineEnds.startType == lowerLineEnds.endType) {
                if (upperLineEnds.startType == Boundary::EDOWN &&
                    lowerLineEnds.startType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMIN, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMIN, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == Boundary::EDOWN &&
                           lowerLineEnds.startType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMAX, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMAX, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                }
            } else if (upperLineEnds.endType == lowerLineEnds.startType) {
                if (upperLineEnds.endType == Boundary::EDOWN &&
                    lowerLineEnds.endType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMIN, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMIN, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::EDOWN &&
                           lowerLineEnds.endType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMAX, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMAX, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                }
            } else {
                SLog(EError, "unexpected condition for case 8 upStart %d upEnd %d loStart %d loEnd %d",
                     upperLineEnds.startType, upperLineEnds.endType, lowerLineEnds.startType,
                     lowerLineEnds.endType);
            }
        } else if (!upperLineEnds.isInvalid && !lowerLineEnds.isInvalid
                   && upperLineEnds.isAdjacent && !lowerLineEnds.isAdjacent) { //case 9

            if (upperLineEnds.startType == lowerLineEnds.startType) {
                if (upperLineEnds.startType == Boundary::EUP &&
                    upperLineEnds.endType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMIN, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMIN, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == Boundary::EUP &&
                           upperLineEnds.endType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMAX, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMAX, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                }
            } else if (upperLineEnds.endType == lowerLineEnds.endType) {
                if (upperLineEnds.endType == Boundary::EUP &&
                    upperLineEnds.startType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMIN, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMIN, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::EUP &&
                           upperLineEnds.startType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMAX, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMAX, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                }
            } else if (upperLineEnds.startType == lowerLineEnds.endType) {
                if (upperLineEnds.startType == Boundary::EUP &&
                    upperLineEnds.endType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMIN, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMIN, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == Boundary::EUP &&
                           upperLineEnds.endType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMAX, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.startType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.end, upperLineEnds.start,
                                          lowerLineEnds.end, lowerLineEnds.start,
                                          Point2(xMAX, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                }
            } else if (upperLineEnds.endType == lowerLineEnds.startType) {
                if (upperLineEnds.endType == Boundary::EUP &&
                    upperLineEnds.startType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMIN, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMIN, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::EUP &&
                           upperLineEnds.startType == Boundary::ERIGHT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMAX, yMAX)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                } else if (upperLineEnds.endType == Boundary::ELEFT) {
                    Point2 vertices[5] = {upperLineEnds.start, upperLineEnds.end,
                                          lowerLineEnds.start, lowerLineEnds.end,
                                          Point2(xMAX, yMIN)};
                    reCamSampled = samplePolygon(vertices, 5, randNumber, sample, pRec.epiLineLen);
                }
            } else {

                SLog(EError, "unexpected condition for case 9 upStart %d upEnd %d loStart %d loEnd %d",
                     upperLineEnds.startType, upperLineEnds.endType, lowerLineEnds.startType,
                     lowerLineEnds.endType);
            }
        } else if (!upperLineEnds.isInvalid && !lowerLineEnds.isInvalid
                   && !upperLineEnds.isAdjacent && !lowerLineEnds.isAdjacent) { // case 10
            if (upperLineEnds.startType == lowerLineEnds.startType) {
                Point2 vertices[4] = {upperLineEnds.end, upperLineEnds.start,
                                      lowerLineEnds.start, lowerLineEnds.end};
                reCamSampled = samplePolygon(vertices, 4, randNumber, sample, pRec.epiLineLen);
            } else {
                Point2 vertices[4] = {upperLineEnds.end, upperLineEnds.start,
                                      lowerLineEnds.end, lowerLineEnds.start};
                reCamSampled = samplePolygon(vertices, 4, randNumber, sample, pRec.epiLineLen);
            }
        } else {
            SLog(EError, "There are more than expected 10 cases");
        }

        Point nearP(reCamSampled.x,reCamSampled.y, 1.f);
        Point resampled = m_projectorToSample(nearP);
        Vector d = normalize(Vector(nearP));

        dRec.d = trafo(d);
        dRec.measure = ESolidAngle;
        dRec.pdf = 1.f / (d.z * d.z * d.z * pRec.epiLineLen);
        pRec.uv = Point2(resampled.x * m_projectorResolution.x, resampled.y * m_projectorResolution.y);

        return Spectrum(m_scale) * pRec.epiLineLen * m_projectorNormalization; // This does not change

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

        Float cosTheta = Frame::cosTheta(d);

        Point planeCoordinate(0.0f,0.0f,1.0f);
        /* Check if the direction points behind the camera */
        if (cosTheta > 0) {
            /* Compute the position on the plane at distance 1 */
            Float invCosTheta = 1.0f / cosTheta;
            planeCoordinate.x = d.x * invCosTheta;
            planeCoordinate.y = d.y * invCosTheta;
        }

        pRec.worldCameraSample = planeCoordinate;
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
        Point epiline = pRec.epiline;
        Vector2 imagePlaneSize = m_projectorImageRect->getExtents();
        Float pixelWidth = imagePlaneSize.x * m_projectorInvResolution.x;

        if (getPoint2LineDistance(p, Point(pRec.epiline.x, pRec.epiline.y, pRec.epiline.z)) > pixelWidth) {
            return 0.0f;
        }

        return importance(d, EPROJ) *  m_projectorNormalization / pRec.epiLineLen;
    }

    Float pdfCameraDirection(const DirectionSamplingRecord &dRec,
                             const PositionSamplingRecord &pRec) const {
        if (dRec.measure != ESolidAngle)
            return 0.0f;

        const Transform &trafo = m_cameraWorldTransform->eval(pRec.time);

        Vector d = trafo.inverse()(dRec.d);
        //TODO: make it possible to calculate fundamental matrix
        //taking in projector points and output camera epipolar line
        //This implementation would make taking on lightImage possible


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
        Point epiline = pRec.epiline;
        Vector2 imagePlaneSize = m_projectorImageRect->getExtents();
        Float pixelWidth = imagePlaneSize.x * m_projectorInvResolution.x;

        if (getPoint2LineDistance(p, Point(pRec.epiline.x, pRec.epiline.y, pRec.epiline.z)) > pixelWidth) {
            return Spectrum(0.0f);
        }
        return importance(d, EPROJ) * Spectrum(m_scale);

    }

    Spectrum evalCameraDirection(const DirectionSamplingRecord &dRec,
                                 const PositionSamplingRecord &pRec) const {
        if (dRec.measure != ESolidAngle)
            return Spectrum(0.0f);

        const Transform &trafo = m_cameraWorldTransform->eval(pRec.time);

        Vector d = trafo.inverse()(dRec.d);
        //TODO: make it possible to calculate fundamental matrix
        //taking in projector points and output camera epipolar line
        //This implementation would make taking on lightImage possible

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

        //TODO: add conditioning to check if the new sampled point
        //fall on the epipolar line

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

        //TODO: add conditioning to check if the new sampled point
        //fall on the epipolar line
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
        oss << "epipolarProbing[" << endl
            << "  disparity = x :" << m_rowTransform << endl
            << "  y :" << m_colTransform << endl
            << "]";
        return oss.str();
    }


    MTS_DECLARE_CLASS()

protected:
    Boundary::EBoundaryType getPointType(Float xMIN, Float xMAX, Float yMIN, Float yMAX, Point2 point) const {
        if (fabs(point.x - xMIN) <= 1e-8) {
            return Boundary::ELEFT;
        } else if (fabs(point.x - xMAX) <= 1e-8) {
            return Boundary::ERIGHT;
        } else if (fabs(point.y - yMIN) <= 1e-8) {
            return Boundary::EUP;
        } else if (fabs(point.y - yMAX) <= 1e-8) {
            return Boundary::EDOWN;
        }

        return Boundary::EINVALID;

    }

    Float getPoint2LineDistance(Point2 p, Point l) const {
        return fabs(p.x * l.x + p.y * l.y + l.z) / sqrt(l.x * l.x + l.y * l.y);
    }

    void getLineEnds(LineEnds &lineEnds, Vector dirTOP, Vector dirBOT, Point epipolarLine) const {

        Float cosThetaTOP = Frame::cosTheta(dirTOP);
        Float cosThetaBOT = Frame::cosTheta(dirBOT);

        Point normCamTOP(0.f, 0.f, 1.f);
        Point normCamBOT(0.f, 0.f, 1.f);
        /* Check if the direction points behind the camera */
        if (cosThetaTOP > 0) {
            /* Compute the position on the plane at distance 1 */
            Float invCosThetaTOP = 1.0f / cosThetaTOP;
            normCamTOP.x = dirTOP.x * invCosThetaTOP;
            normCamTOP.y = dirTOP.y * invCosThetaTOP;
        }

        if (cosThetaTOP > 0) {
            /* Compute the position on the plane at distance 1 */
            Float invCosThetaBOT = 1.0f / cosThetaBOT;
            normCamBOT.x = dirBOT.x * invCosThetaBOT;
            normCamBOT.y = dirBOT.y * invCosThetaBOT;
        }

        Float yMIN = normCamTOP.y < normCamBOT.y ? normCamTOP.y : normCamBOT.y;
        Float yMAX = normCamTOP.y < normCamBOT.y ? normCamBOT.y : normCamTOP.y;
        Float xMIN = normCamTOP.x < normCamBOT.x ? normCamTOP.x : normCamBOT.x;
        Float xMAX = normCamTOP.x < normCamBOT.x ? normCamBOT.x : normCamTOP.x;


        double yB1;
        double yB2;
        Point2 ysmaller;
        Point2 ylarger;


        //yB1: y coordinate of line's intersection with x = xMAX,
        //yB2: y coordinate of line's intersection with x= xMIN
        if (fabs(epipolarLine.y) >= 1e-9) {
            assert(fabs(epipolarLine.y) >= 1e-9);
            yB1 = (-epipolarLine.z - xMAX * epipolarLine.x) / epipolarLine.y;
            yB2 = (-epipolarLine.z - xMIN * epipolarLine.x) / epipolarLine.y;
            ysmaller = yB1 < yB2 ? Point2(xMAX, yB1) : Point2(xMIN, yB2);
            ylarger = yB1 < yB2 ? Point2(xMIN, yB2) : Point2(xMAX, yB1);
        } else {
            assert(fabs(epipolarLine.y) < 1e-9);
            if (epipolarLine.x < 1e-9) {
                //TODO: change this condition to make it possible for sampling over the whole plane
                lineEnds.startType = Boundary::EINVALID;
                lineEnds.endType = Boundary::EINVALID;
                lineEnds.isInvalid;
                return;
            }

            //check if two intersection points lie  within the ybounding box
            yB1 = (-epipolarLine.z / epipolarLine.x >= xMIN && -epipolarLine.z / epipolarLine.x < xMAX)
                  ? -epipolarLine.z / epipolarLine.x : yMIN - 1.f;
            yB2 = (-epipolarLine.z / epipolarLine.x >= xMIN && -epipolarLine.z / epipolarLine.x < xMAX)
                  ? -epipolarLine.z / epipolarLine.x : yMIN - 2.f;
            ysmaller = yB1 < yB2 ? Point2(-epipolarLine.z / epipolarLine.x, yB1) : Point2(
                    -epipolarLine.z / epipolarLine.x, yB2);
            ylarger = yB1 < yB2 ? Point2(-epipolarLine.z / epipolarLine.x, yB2) : Point2(
                    -epipolarLine.z / epipolarLine.x, yB1);
        }

        if (ylarger.y < yMIN || ysmaller.y > yMAX) {
            lineEnds.startType = Boundary::EINVALID;
            lineEnds.endType = Boundary::EINVALID;
            lineEnds.isInvalid = true;
            return;
        }

        //default case y = a, where x coefficient = 0
        //we set both x intersection to be out of bound.
        double xB1 = xMAX + 10.f;
        double xB2 = xMIN - 5.f;
        if (fabs(epipolarLine.x) >= 1e-7) {
            assert(fabs(epipolarLine.x) >= 1e-7);
            xB1 = (-epipolarLine.z - yMAX * epipolarLine.y) / epipolarLine.x;
            xB2 = (-epipolarLine.z - yMIN * epipolarLine.y) / epipolarLine.x;
        }
        Point2 xsmaller = xB1 < xB2 ? Point2(xB1, yMAX) : Point2(xB2, yMIN);
        Point2 xlarger = xB1 < xB2 ? Point2(xB2, yMIN) : Point2(xB1, yMAX);

        Point2 start;
        Point2 end;


        if (xlarger.x < xMIN) {
            start = ylarger.x < xMAX ? ylarger : ysmaller;
            end = ylarger.x < xMAX ? ysmaller : ylarger;
        } else if (xsmaller.x < xMIN && xlarger.x <= xMAX) {
            start = ylarger.x < xMAX ? ylarger : ysmaller;
            end = xlarger;
        } else if (xsmaller.x >= xMIN && xlarger.x <= xMAX) {
            start = xsmaller;
            end = xlarger;
        } else if (xsmaller.x >= xMIN && xsmaller.x <= xMAX && xlarger.x > xMAX) {
            start = xsmaller;
            end = ylarger.x < xMAX ? ysmaller : ylarger;
        } else {
            start = ylarger.x < xMAX ? ylarger : ysmaller;
            end = ylarger.x < xMAX ? ysmaller : ylarger;
        }

        lineEnds.start = start;
        lineEnds.end = end;

        lineEnds.startType = getPointType(xMIN, xMAX, yMIN, yMAX, start);
        lineEnds.endType = getPointType(xMIN, xMAX, yMIN, yMAX, end);
        if(lineEnds.startType == lineEnds.endType){
            lineEnds.isInvalid = true;
            return;
        }

        lineEnds.isAdjacent = Boundary::isAdjacent(lineEnds.startType, lineEnds.endType);
        lineEnds.isInvalid = false;

        if (lineEnds.isAdjacent) {
            Boundary::getRelatedBoxCorner(lineEnds.startType, lineEnds.endType,
                                          xMIN, xMAX, yMIN, yMAX, lineEnds.corner);
        }
        return;
    }

    Point2 sampleTriangle(Point2 p0, Point2 p1, Point2 p2, Point2 sample) const {
        float r1 = sample.x;
        float r2 = sample.y;
        Point2 sampleHit = (1.f - sqrtf(r1)) * p0 + (sqrtf(r1) * (1.0f - r2)) * p1 + sqrtf(r1) * r2 * p2;
        return sampleHit;
    }

    Float getTriangArea(Point2 p0, Point2 p1, Point2 p2) const {
        Vector vec1 = Vector((p1 - p0).x, (p1 - p0).y, 0.f);
        Vector vec2 = Vector((p2 - p0).x, (p2 - p0).y, 0.f);
        return cross(vec1, vec2).length() / 2.f;
    }

    Point2 samplePolygon(Point2 *vertices, int32_t n, float randNumber, Point2 sample, float &epiArea) const {

        if (n < 3) {
            SLog(EError, "less than three vertices for sampling in a polygon");
        }

        //Create cdf for Triangles area distribution
        float areas[n - 1];
        Point2 v1, v2, v3;

        //cdf starts with 0 probability
        areas[0] = 0.f;
        areas[1] = getTriangArea(vertices[0], vertices[1], vertices[2]);
        float totalArea = areas[1];
        for (int i = 2; i < n - 1; i++) {
            areas[i] = areas[i - 1] + getTriangArea(vertices[0], vertices[i], vertices[i + 1]);
        }
        epiArea = areas[n - 2];
        totalArea = areas[n - 2];

        //Normalize the distriution
        if (fabs(totalArea) < 1e-7) {
            for (int i = 1; i < n - 1; i++) {
                areas[i] = 0.f;
            }
        } else {
            for (int i = 1; i < n - 1; i++) {
                areas[i] = areas[i] / totalArea;
            }
            areas[n - 2] = 1.f;
        }
        uint32_t whichTriangle = sampleReuse(areas, (uint32_t) n, randNumber);

        return sampleTriangle(vertices[0], vertices[whichTriangle], vertices[whichTriangle + 1], sample);

    }

private:
    /// Sample from an array using the inversion method
    inline uint32_t sampleReuse(float *cdf, uint32_t size, Float &sample) const {
        float *entry = std::lower_bound(cdf, cdf + size + 1, (float) sample);
        uint32_t index = std::min((uint32_t) std::max((ptrdiff_t) 0, entry - cdf - 1), size - 1);
        sample = (sample - (Float) cdf[index]) / (Float) (cdf[index + 1] - cdf[index]);
        return index;
    }

    Point3 m_worldCameraOrigin;
    Point3 m_worldCameraZAxis;
    Point3 m_worldCameraYAxis;
    Point3 m_worldCameraXAxis;
    Matrix3x3 funda;
};

MTS_IMPLEMENT_CLASS_S(EpipolarProbing, false, BasicProbe)
MTS_EXPORT_PLUGIN(EpipolarProbing, "Epipolar Probing Pattern");

MTS_NAMESPACE_END

