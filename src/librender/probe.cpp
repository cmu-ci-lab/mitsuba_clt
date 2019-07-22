//
// Created by Jiatian Sun on 8/31/18.
//

#include <mitsuba/render/probe.h>
//#include <mitsuba/core/plugin.h>
//#include <boost/algorithm/string.hpp>

#include <iostream>

MTS_NAMESPACE_BEGIN

Probe::Probe(const Properties &props) : ConfigurableObject(props){
    Vector2i disp = props.getDisparity("disparity", Vector2i(0,0));

    m_colTransform = disp.x;
    m_rowTransform = disp.y;
}

/// Unserialize a emitter instance from a binary data stream
Probe::Probe(Stream *stream, InstanceManager *manager): ConfigurableObject(stream, manager) {
    m_type = stream->readString();
    m_colTransform = stream->readInt();
    m_rowTransform = stream->readInt();
    for(int r = 0; r < 3; r++){
        for(int c = 0; c< 3; c++){
            m_epipolarMatrix(r,c) = stream->readFloat();
        }
    }
}

Probe::~Probe() {}

void Probe::serialize(Stream *stream, InstanceManager *manager) const {
    ConfigurableObject::serialize(stream, manager);
    stream->writeString(m_type);
    stream->writeFloat(m_colTransform);
    stream->writeFloat(m_rowTransform);
    for(int r = 0; r < 3; r++){
        for(int c = 0; c< 3; c++){
            stream->writeFloat(m_epipolarMatrix(r,c));
        }
    }
}


void Probe::readFromCamera(Stream *stream){

}

void Probe::readFromProjector(Stream *stream){

}

void Probe::configureAfterLoadingCamProj(){

}

void Probe::addChild(const std::string &name, ConfigurableObject *child) {
    ConfigurableObject::addChild(name, child);
}

Spectrum Probe::sampleCameraPosition(PositionSamplingRecord &pRec,
                                         const Point2 &sample, const Point2 *extra) const {
    NotImplementedError("sampleCameraPosition");
}

Spectrum Probe::sampleCameraDirection(DirectionSamplingRecord &dRec,
                                          PositionSamplingRecord &pRec, const Point2 &sample,
                                          const Point2 *extra) const {
    NotImplementedError("sampleCameraDirection");
}

Spectrum Probe::sampleCameraDirect(DirectSamplingRecord &dRec, const Point2 &sample) const {
    NotImplementedError("sampleCameraDirect");
}

Spectrum Probe::evalCameraPosition(const PositionSamplingRecord &pRec) const {
    NotImplementedError("evalCameraPosition");
}

Spectrum Probe::evalCameraDirection(const DirectionSamplingRecord &dRec,
                                        const PositionSamplingRecord &pRec) const {
    NotImplementedError("evalCameraDirection");
}

Float Probe::pdfCameraPosition(const PositionSamplingRecord &pRec) const {
    NotImplementedError("pdfCameraPosition");
}

Float Probe::pdfCameraDirection(const DirectionSamplingRecord &dRec,
                                    const PositionSamplingRecord &pRec) const {
    NotImplementedError("pdfCameraDirection");
}

Float Probe::pdfCameraDirect(const DirectSamplingRecord &dRec) const {
    NotImplementedError("pdfCameraDirect");
}

Spectrum Probe::sampleProjectorPosition(PositionSamplingRecord &pRec, Sampler *sampler,
                                     const Point2 &sample, const Point2 *extra) const {
    NotImplementedError("sampleProjectorPosition");
}

Spectrum Probe::sampleProjectorDirection(DirectionSamplingRecord &dRec,
                                      PositionSamplingRecord &pRec, Sampler *sampler,
                                      const Point2 &sample,
                                      const Point2 *extra) const {
    NotImplementedError("sampleDirection");
}

Spectrum Probe::sampleProjectorDirect(DirectSamplingRecord &dRec, const Point2 &sample) const {
    NotImplementedError("sampleDirect");
}

Spectrum Probe::evalProjectorPosition(const PositionSamplingRecord &pRec) const {
    NotImplementedError("evalPosition");
}

Spectrum Probe::evalProjectorDirection(const DirectionSamplingRecord &dRec,
                                    const PositionSamplingRecord &pRec) const {
    NotImplementedError("evalDirection");
}

Float Probe::pdfProjectorPosition(const PositionSamplingRecord &pRec) const {
    NotImplementedError("pdfPosition");
}

Float Probe::pdfProjectorDirection(const DirectionSamplingRecord &dRec,
                                const PositionSamplingRecord &pRec) const {
    NotImplementedError("pdfDirection");
}

Float Probe::pdfProjectorDirect(const DirectSamplingRecord &dRec) const {
    NotImplementedError("pdfDirect");
}

void BasicProbe::readFromCamera(Stream *stream){
    Probe::readFromProjector(stream);
    m_cameraNearClip = stream->readFloat();
    m_cameraFarClip = stream->readFloat();
    m_cameraNormalization = stream->readFloat();
    m_cameraResolution = Vector2(stream);
    m_cameraWorldTransform = new AnimatedTransform(stream);
    m_cameraToSample = Transform(stream);
    m_sampleToCamera = Transform(stream);
    m_cameraImageRect = new AABB2(stream);
    m_cameraInvResolution = Vector2(1.f/m_cameraResolution.x,1.f/m_cameraResolution.y);
    m_cameraSize = Vector2i(math::floorToInt(m_cameraResolution.x),
            math::floorToInt(m_cameraResolution.y));
}

void BasicProbe::readFromProjector(Stream *stream){
    Probe::readFromProjector(stream);
    m_scale = stream->readFloat();
    m_projectorNearClip = stream->readFloat();
    m_projectorFarClip = stream->readFloat();
    m_projectorNormalization = stream->readFloat();
    m_projectorResolution = Vector2(stream);
    m_projectorWorldTransform = new AnimatedTransform(stream);
    m_projectorToSample = Transform(stream);
    m_sampleToProjector = Transform(stream);
    m_projectorImageRect = new AABB2(stream);
    m_projectorInvResolution = Vector2(1.f/m_projectorResolution.x,1.f/m_projectorResolution.y);
    m_projectorSize = Vector2i(math::floorToInt(m_projectorResolution.x),
            math::floorToInt(m_projectorResolution.y));
}

void BasicProbe::configureAfterLoadingCamProj(){
    Probe::configureAfterLoadingCamProj();

}

BasicProbe::BasicProbe(const Properties &props) : Probe(props){
}

/// Unserialize a basic probing pattern instance from a binary data stream
BasicProbe::BasicProbe(Stream *stream, InstanceManager *manager): Probe(stream, manager) {
}

BasicProbe::~BasicProbe() {}


MTS_IMPLEMENT_CLASS(BasicProbe, true, Probe)
MTS_IMPLEMENT_CLASS(Probe, true, ConfigurableObject)
MTS_NAMESPACE_END


