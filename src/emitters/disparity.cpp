
#include <mitsuba/render/sensor.h>
#include <mitsuba/render/medium.h>
#include <mitsuba/core/track.h>
#include <mitsuba/core/frame.h>

MTS_NAMESPACE_BEGIN

  class DisparityEmitter : public PerspectiveEmitter {
  public:
      typedef TSpectrum<half, SPECTRUM_SAMPLES> SpectrumHalf;
      typedef TMIPMap<Spectrum, SpectrumHalf> MIPMap;
    DisparityEmitter(const Properties &props) : PerspectiveEmitter(props) {
              m_type |=  EDeltaPosition | EPerspectiveEmitter | EOnSurface | EDirectionSampleMapsToPixels;

              m_scale = props.getFloat("scale", 1.0f);
              m_width = props.getFloat("width", 1.0f);
              m_height = props.getFloat("height", 1.0f);
              m_rowTransform = props.getInteger("rowTransform", 0);
              m_colTransform = props.getInteger("colTransform", 0);
              if (props.getTransform("toWorld", Transform()).hasScale())
              Log(EError, "Scale factors in the emitter-to-world "
                      "transformation are not allowed!");
      }

    DisparityEmitter(Stream *stream, InstanceManager *manager)
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
        m_size.x = m_width;
        m_size.y = m_height;
        m_aspect = (Float)m_size.x / (Float) m_size.y;

        m_resolution = Vector2(m_size.x,m_size.y);
        m_invResolution = Vector2(
                (Float) 1 / m_resolution.x,
                (Float) 1 / m_resolution.y);
        m_rowTransform = 0;

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
        std::cout<<"all white projector \n"<<m_resolution.toString()<<endl;

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
               * invCosTheta * invCosTheta * (Float)m_resolution.x;
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
      return (pRec.measure == EDiscrete) ? 1.0f : 0.0f;
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

      pRec.uv = Point2(samplePos.x * m_resolution.x,
                       samplePos.y * m_resolution.y);

      /* Compute the corresponding position on the
         near plane (in local camera space) */
      Point nearP = m_sampleToCamera(samplePos);

      /* Turn that into a normalized ray direction */
      Vector d = normalize(Vector(nearP));
      dRec.d = trafo(d);
      dRec.measure = ESolidAngle;
      dRec.pdf = m_normalization / (d.z * d.z * d.z);
      dRec.planePosition = Point2(pRec.uv);

//      std::string str = "em " + pRec.uv.toString();
//      std::cout<<str<<endl;
      return Spectrum(m_scale);
    }

    Float pdfDirection(const DirectionSamplingRecord &dRec,
                       const PositionSamplingRecord &pRec) const {
      if (dRec.measure != ESolidAngle)
        return 0.0f;

      const Transform &trafo = m_worldTransform->eval(pRec.time);

      return importance(trafo.inverse()(dRec.d));
    }

    Spectrum evalDirection(const DirectionSamplingRecord &dRec,
                           const PositionSamplingRecord &pRec) const {
      if (dRec.measure != ESolidAngle)
        return Spectrum(0.0f);

      const Transform &trafo = m_worldTransform->eval(pRec.time);

      Vector v = trafo.inverse()(dRec.d);
      Float cosTheta = Frame::cosTheta(v);

      /* Check if the direction points behind the camera */
      if (cosTheta <= 0)
        return Spectrum(0.0f);

//      /* Compute the position on the plane at distance 1 */
//      Float invCosTheta = 1.0f / cosTheta;
//      Point2 uv(v.x * invCosTheta, v.y * invCosTheta);
//
//      /* Check if the point lies inside the chosen crop rectangle */
//      if (!m_imageRect.contains(uv)) {
//        return Spectrum(0.0f);
//      }
//
//      Point samplePoint(uv.x,uv.y,1.0f);
//      Point sample = m_cameraToSample(samplePoint * m_nearClip);
//
//      printf("emitter %f\n",pRec.uv.x);
//      return Spectrum(sample.x);

//      pRec.disparitySample = pRec.uv.x;

//      return Spectrum(pRec.uv.x);
      return importance(v) * Spectrum(m_scale);
    }


    Spectrum sampleRay(Ray &ray, const Point2 &pixelSample,
                         const Point2 &otherSample, Float timeSample) const {
        ray.setTime(timeSample);

        /* Compute the corresponding position on the
           near plane (in local emitter space) */
        Point nearP = m_sampleToCamera(Point(
                pixelSample.x * m_invResolution.x,
                pixelSample.y * m_invResolution.y, 0.0f));

        /* Turn that into a normalized ray direction, and
           adjust the ray interval accordingly */
        Vector d = normalize(Vector(nearP));
        Float invZ = 1.0f / d.z;
        ray.mint = m_nearClip * invZ;
        ray.maxt = m_farClip * invZ;

        const Transform &trafo = m_worldTransform->eval(ray.time);
        ray.setOrigin(trafo.transformAffine(Point(0.0f)));
        ray.setDirection(trafo(d));

        return Spectrum(m_scale)/(Float)m_resolution.x;
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
//            std::cout<<screenSample.toString()<<endl;
        dRec.uv = Point2(screenSample.x, screenSample.y);
        if (dRec.uv.x < 0 || dRec.uv.x > 1 ||
            dRec.uv.y < 0 || dRec.uv.y > 1) {
          dRec.pdf = 0.0f;
          return Spectrum(0.0f);
        }

        dRec.uv.x *= m_resolution.x;
        dRec.uv.y *= m_resolution.y;

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

        return Spectrum(importance(localD) * invDist * invDist * m_scale)/(Float)m_resolution.x;

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
      Float m_scale;
      Float m_width;
      Float m_height;
      Float m_invSurfaceArea;
      Transform m_cameraToSample;
      Transform m_sampleToCamera;
      AABB2 m_imageRect;
      Float m_normalization;

    };

    MTS_IMPLEMENT_CLASS_S(DisparityEmitter, false, PerspectiveEmitter)
    MTS_EXPORT_PLUGIN(DisparityEmitter, "disparity emitter");

  MTS_NAMESPACE_END