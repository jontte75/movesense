#include "SumVectorService.h"
#include "app-resources/resources.h"
#include "common/core/debug.h"
#include "meas_acc/resources.h"

#include <float.h>

#define ACC_PATH_WITH_SAMPLERATE "Meas/Acc/13"

const char* const SumVectorService::LAUNCHABLE_NAME = "Sumvector";

static const wb::ExecutionContextId sExecutionContextId =
    WB_RES::LOCAL::EXERCISE_SUMVECTOR_JONI::EXECUTION_CONTEXT;

static const wb::LocalResourceId sProviderResources[] =
{
    WB_RES::LOCAL::EXERCISE_SUMVECTOR_JONI::LID,
};

SumVectorService::SumVectorService()
    : ResourceClient(WBDEBUG_NAME(__FUNCTION__), sExecutionContextId),
      ResourceProvider(WBDEBUG_NAME(__FUNCTION__), sExecutionContextId),
      LaunchableModule(LAUNCHABLE_NAME, sExecutionContextId),
      mOngoingRequests()
{
}

SumVectorService::~SumVectorService()
{
}

bool SumVectorService::initModule()
{
    if (registerProviderResources(sProviderResources) != wb::HTTP_CODE_OK)
    {
        return false;
    }

    mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    return true;
}

void SumVectorService::deinitModule()
{
    unregisterProviderResources(sProviderResources);
    mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
}

bool SumVectorService::startModule()
{
    mModuleState = WB_RES::ModuleStateValues::STARTED;
	return true;
}

void SumVectorService::stopModule()
{
    stopRunning();
    mModuleState = WB_RES::ModuleStateValues::STOPPED;
}

void SumVectorService::onUnsubscribeResult(wb::RequestId requestId,
                                                     wb::ResourceId resourceId,
                                                     wb::Result resultCode,
                                                     const wb::Value& rResultData)
{
    DEBUGLOG("SumVectorService::onUnsubscribeResult() called.");
}

void SumVectorService::onSubscribeResult(wb::RequestId requestId,
                                                   wb::ResourceId resourceId,
                                                   wb::Result resultCode,
                                                   const wb::Value& rResultData)
{
    wb::Request relatedIncomingRequest;
    bool relatedRequestFound = mOngoingRequests.get(requestId, relatedIncomingRequest);

    if (relatedRequestFound)
    {
        returnResult(relatedIncomingRequest, wb::HTTP_CODE_OK);
    }
}

wb::Result SumVectorService::startRunning(wb::RequestId& remoteRequestId)
{
    if (isRunning)
    {
        return wb::HTTP_CODE_OK;
    }

    // Reset max acceleration members
    mSamplesIncluded = 0;
	mSumValue.mX = 0;
	mSumValue.mY = 0;
	mSumValue.mZ = 0;

    wb::Result result = getResource(ACC_PATH_WITH_SAMPLERATE, mMeasAccResourceId);
    if (!wb::RETURN_OKC(result))
    {
		DEBUGLOG("JKo: not ok: %u", result);
        return result;
    }

    result = asyncSubscribe(mMeasAccResourceId, AsyncRequestOptions(&remoteRequestId, 0, true));

    if (!wb::RETURN_OKC(result))
    {
        DEBUGLOG("asyncSubscribe threw error: %u", result);
        return wb::HTTP_CODE_BAD_REQUEST;
    }
    isRunning = true;

    return wb::HTTP_CODE_OK;
}

wb::Result SumVectorService::stopRunning()
{
    if (!isRunning)
    {
        return wb::HTTP_CODE_OK;
    }

    if (isResourceSubscribed(WB_RES::LOCAL::EXERCISE_SUMVECTOR_JONI::ID) == wb::HTTP_CODE_OK)
    {
        return wb::HTTP_CODE_OK;
    }

    // Unsubscribe the LinearAcceleration resource, when unsubscribe is done, we get callback
    wb::Result result = asyncUnsubscribe(mMeasAccResourceId, NULL);
    if (!wb::RETURN_OKC(result))
    {
        DEBUGLOG("asyncUnsubscribe threw error: %u", result);
    }
    isRunning = false;
    releaseResource(mMeasAccResourceId);

    return wb::HTTP_CODE_OK;
}

void SumVectorService::onNotify(wb::ResourceId resourceId,
                                const wb::Value& value,
                                const wb::ParameterList& parameters)
{
    // Confirm that it is the correct resource
    switch (resourceId.localResourceId)
    {
    case WB_RES::LOCAL::MEAS_ACC_SAMPLERATE::LID:
    {
        const WB_RES::AccData& linearAccelerationValue =
            value.convertTo<const WB_RES::AccData&>();

        if (linearAccelerationValue.arrayAcc.size() <= 0)
        {
            // No value, do nothing...
			return;
        }

        const wb::Array<wb::FloatVector3D>& arrayData = linearAccelerationValue.arrayAcc;

        uint32_t relativeTime = linearAccelerationValue.timestamp;

        for (size_t i = 0; i < arrayData.size(); i++)
        {
            mSamplesIncluded++;

            wb::FloatVector3D accValue = arrayData[i];
			mSumValue += accValue;
		}

        // 13Hz --> 1 s
        if (mSamplesIncluded >= 13)
        {
            // Reset counter and notify our subscribers
			wb::FloatVector3D avValue;
			avValue = mSumValue;
			avValue /= mSamplesIncluded;

            WB_RES::SumVectorAverageData sumVectAvData;
			sumVectAvData.relativeTime = relativeTime;
			sumVectAvData.sumVector.x = avValue.mX;
			sumVectAvData.sumVector.y = avValue.mY;
			sumVectAvData.sumVector.z = avValue.mZ;
            // Reset members
            mSamplesIncluded = 0;
			mSumValue.mX = 0;
			mSumValue.mY = 0;
			mSumValue.mZ = 0;
            // and update our WB resource. This causes notification to be fired to our subscribers
            updateResource(WB_RES::LOCAL::EXERCISE_SUMVECTOR_JONI(), ResponseOptions::Empty, sumVectAvData);
			blinkLed();
        }
        break;
    }
    }
}

void SumVectorService::onSubscribe(const wb::Request& request,
                                             const wb::ParameterList& parameters)
{
    switch (request.getResourceId().localResourceId)
    {
    case WB_RES::LOCAL::EXERCISE_SUMVECTOR_JONI::LID:
    {
        // Someone subscribed to our service. Start collecting data and notifying when our service changes state (every 10 seconds)
        wb::RequestId remoteRequestId;
        wb::Result result = startRunning(remoteRequestId);

        if (isRunning)
        {
            returnResult(request, wb::HTTP_CODE_OK);
            return;
        }

        if (!wb::RETURN_OK(result))
        {
            returnResult(request, result);
            return;
        }
        bool queueResult = mOngoingRequests.put(remoteRequestId, request);
        (void)queueResult;
        WB_ASSERT(queueResult);
        break;
    }
    default:
        ASSERT(0); // Should not happen
    }
}

void SumVectorService::onUnsubscribe(const wb::Request& request,
                                               const wb::ParameterList& parameters)
{
	DEBUGLOG("JKo: onUnsubscribe called.");
    switch (request.getResourceId().localResourceId)
    {
    case WB_RES::LOCAL::EXERCISE_SUMVECTOR_JONI::LID:
        stopRunning();
        returnResult(request, wb::HTTP_CODE_OK);
        break;

    default:
        ASSERT(0); // Should not happen
    }
}

void SumVectorService::onRemoteWhiteboardDisconnected(wb::WhiteboardId whiteboardId)
{
    stopRunning();
}

void SumVectorService::onClientUnavailable(wb::ClientId clientId)
{
    stopRunning();
}

void SumVectorService::blinkLed()
{
	const WB_RES::VisualIndType type = WB_RES::VisualIndTypeValues::SHORT_VISUAL_INDICATION; // defined in ui/ind.yaml
	asyncPut(WB_RES::LOCAL::UI_IND_VISUAL(), AsyncRequestOptions::Empty, type); // PUT request to trigger led blink
	// ignore the immediate and asynchronous Results as this is not critical
}