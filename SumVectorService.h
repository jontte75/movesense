#pragma once

#include <whiteboard/LaunchableModule.h>
#include <whiteboard/ResourceClient.h>
#include <whiteboard/ResourceProvider.h>
#include <whiteboard/containers/RequestMap.h>

#include "wb-resources/resources.h"
#include "ui_ind/resources.h"

class SumVectorService FINAL : private wb::ResourceClient,
							   private wb::ResourceProvider,
							   public wb::LaunchableModule

{
public:
	/** Name of this class. Used in StartupProvider list. */
	static const char* const LAUNCHABLE_NAME;
	SumVectorService();
	~SumVectorService();

private:
	/** @see whiteboard::ILaunchableModule::initModule */
	virtual bool initModule() OVERRIDE;
	/** @see whiteboard::ILaunchableModule::deinitModule */
	virtual void deinitModule() OVERRIDE;
	/** @see whiteboard::ILaunchableModule::startModule */
	virtual bool startModule() OVERRIDE;
	/** @see whiteboard::ILaunchableModule::stopModule */
	virtual void stopModule() OVERRIDE;

	/** @see whiteboard::ResourceProvider::onSubscribe */
	virtual void onSubscribe(const wb::Request& request,
		const wb::ParameterList& parameters) OVERRIDE;

	/** @see whiteboard::ResourceProvider::onUnsubscribe */
	virtual void onUnsubscribe(const wb::Request& request,
		const wb::ParameterList& parameters) OVERRIDE;

	/** @see whiteboard::ResourceProvider::onRemoteWhiteboardDisconnected */
	virtual void onRemoteWhiteboardDisconnected(wb::WhiteboardId whiteboardId) OVERRIDE;

	/** @see whiteboard::ResourceProvider::onClientUnavailable */
	virtual void onClientUnavailable(wb::ClientId clientId) OVERRIDE;

	/** @see whiteboard::ResourceClient::onNotify */
	virtual void onNotify(wb::ResourceId resourceId, const wb::Value& value,
		const wb::ParameterList& parameters);

	/** @see whiteboard::ResourceClient::onSubscribeResult */
	virtual void onSubscribeResult(wb::RequestId requestId,
		wb::ResourceId resourceId,
		wb::Result resultCode,
		const wb::Value& rResultData);

	/** @see whiteboard::ResourceClient::onUnsubscribeResult */
	virtual void onUnsubscribeResult(wb::RequestId requestId,
		wb::ResourceId resourceId,
		wb::Result resultCode,
		const wb::Value& rResultData);

	wb::Result startRunning(wb::RequestId& remoteRequestId);
	wb::Result stopRunning();
	void blinkLed();

private:
	wb::RequestMap<2, void> mOngoingRequests; // For storing relations of incoming & outgoing requests
	wb::ResourceId mMeasAccResourceId{ wb::ID_INVALID_RESOURCE };
	bool isRunning{ false };
	size_t mSamplesIncluded{ 0 };
	wb::FloatVector3D mSumValue{ 0,0,0 };
};
