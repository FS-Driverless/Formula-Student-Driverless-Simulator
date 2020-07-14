#include "AirSimGameMode.h"
#include "Misc/FileHelper.h"
#include "IImageWrapperModule.h"
#include "SimHUD/SimHUD.h"
#include "common/Common.hpp"
#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "Vehicles/Car/SimModeCar.h"


class AUnrealLog : public msr::airlib::Utils::Logger
{
public:
    virtual void log(int level, const std::string& message) override 
    {
        size_t tab_pos;
        static const std::string delim = ":\t";
        if ((tab_pos = message.find(delim)) != std::string::npos) {
            UAirBlueprintLib::LogMessageString(message.substr(0, tab_pos), 
                message.substr(tab_pos + delim.size(), std::string::npos), LogDebugLevel::Informational);
            
            return; //display only
        }

        if (level == msr::airlib::Utils::kLogLevelError) {
            UE_LOG(LogTemp, Error, TEXT("%s"), *FString(message.c_str()));
        }
        else if (level == msr::airlib::Utils::kLogLevelWarn) {
            UE_LOG(LogTemp, Warning, TEXT("%s"), *FString(message.c_str()));
        }
        else {
            UE_LOG(LogTemp, Log, TEXT("%s"), *FString(message.c_str()));
        }
  
        //also do default logging
        msr::airlib::Utils::Logger::log(level, message);
    }
};

static AUnrealLog GlobalASimLog;

AAirSimGameMode::AAirSimGameMode(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    DefaultPawnClass = nullptr;
    static ConstructorHelpers::FClassFinder<APawn> AirsimSpectatorPawn(TEXT("/AirSim/AirsimSpectatorPawn"));
    SpectatorClass = AirsimSpectatorPawn.Class;
    // DefaultPawnClass = ASpectatorPawn::StaticClass();
    HUDClass = ASimHUD::StaticClass();

    common_utils::Utils::getSetLogger(&GlobalASimLog);

    //module loading is not allowed outside of the main thread, so we load the ImageWrapper module ahead of time.
    static IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
}

void AAirSimGameMode::BeginPlay() 
{
    try
    {
        UAirBlueprintLib::OnBeginPlay();
        initializeSettings();
        setUnrealEngineSettings();
        createSimMode();

        if (simmode)
            simmode->startApiServer();
        else
            UAirBlueprintLib::LogMessageString("Error at startup: ", "simmode could not be created", LogDebugLevel::Failure);
    }
    catch (std::exception &ex)
    {
        UAirBlueprintLib::LogMessageString("Error at startup: ", ex.what(), LogDebugLevel::Failure);
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok, std::string("Error at startup: ") + ex.what(), "Error");
    }

    Super::BeginPlay();
}

void AAirSimGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (simmode)
        simmode->stopApiServer();

    if (simmode)
    {
        simmode->Destroy();
        simmode = nullptr;
    }

    UAirBlueprintLib::OnEndPlay();

    Super::EndPlay(EndPlayReason);
}

void AAirSimGameMode::PostLogin(APlayerController * newPlayer) {
    newPlayer->StartSpectatingOnly();
}

void AAirSimGameMode::initializeSettings()
{
    std::string settingsText;
    readSettingsTextFromFile(FString(common_utils::FileSystem::getConfigFilePath().c_str()), settingsText);
    msr::airlib::AirSimSettings::initializeSettings(settingsText);

    msr::airlib::AirSimSettings::singleton().load();
    for (const auto &warning : msr::airlib::AirSimSettings::singleton().warning_messages)
    {
        UAirBlueprintLib::LogMessageString(warning, "", LogDebugLevel::Failure);
    }
    for (const auto &error : msr::airlib::AirSimSettings::singleton().error_messages)
    {
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok, error, "settings.json");
    }
}

void AAirSimGameMode::readSettingsTextFromFile(FString settingsFilepath, std::string &settingsText)
{
    if (!FPaths::FileExists(settingsFilepath)) {
        throw std::runtime_error("settings.json file does not exist. Ensure the ~/Formula-Student-Driverless-Simulator/settings.json file exists.");
    }
    FString settingsTextFStr;
    if (FFileHelper::LoadFileToString(settingsTextFStr, *settingsFilepath))
    {
        UAirBlueprintLib::LogMessageString("Loaded settings from ", TCHAR_TO_UTF8(*settingsFilepath), LogDebugLevel::Informational);
        settingsText = TCHAR_TO_UTF8(*settingsTextFStr);
    }
    else
    {
        UAirBlueprintLib::LogMessageString("Cannot read settings.json file ", TCHAR_TO_UTF8(*settingsFilepath), LogDebugLevel::Failure);
        throw std::runtime_error("Failed reading settings.json. Ensure the ~/Formula-Student-Driverless-Simulator/settings.json file is correct.");
    }
}

void AAirSimGameMode::setUnrealEngineSettings()
{
    //TODO: should we only do below on SceneCapture2D components and cameras?
    //avoid motion blur so capture images don't get
    GetWorld()->GetGameViewport()->GetEngineShowFlags()->SetMotionBlur(false);

    //use two different methods to set console var because sometime it doesn't seem to work
    static const auto custom_depth_var = IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth"));
    custom_depth_var->Set(3);

    //Equivalent to enabling Custom Stencil in Project > Settings > Rendering > Postprocessing
    UKismetSystemLibrary::ExecuteConsoleCommand(GetWorld(), FString("r.CustomDepth 3"));

    //during startup we init stencil IDs to random hash and it takes long time for large environments
    //we get error that GameThread has timed out after 30 sec waiting on render thread
    static const auto render_timeout_var = IConsoleManager::Get().FindConsoleVariable(TEXT("g.TimeoutForBlockOnRenderFence"));
    render_timeout_var->Set(300000);
}

void AAirSimGameMode::createSimMode()
{
    FActorSpawnParameters simmode_spawn_params;
    simmode_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    //spawn at origin. We will use this to do global NED transforms, for ex, non-vehicle objects in environment

    simmode = this->GetWorld()->SpawnActor<ASimModeCar>(FVector::ZeroVector, FRotator::ZeroRotator, simmode_spawn_params);
}
