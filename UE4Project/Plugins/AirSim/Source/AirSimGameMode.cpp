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

void AAirSimGameMode::StartPlay() 
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

    Super::StartPlay();
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

std::string AAirSimGameMode::getSimModeFromUser()
{
    return "Car";
}

void AAirSimGameMode::initializeSettings()
{
    std::string settingsText;
    if (getSettingsText(settingsText))
        msr::airlib::AirSimSettings::initializeSettings(settingsText);
    else
       msr::airlib:: AirSimSettings::createDefaultSettingsFile();

    msr::airlib::AirSimSettings::singleton().load(std::bind(&AAirSimGameMode::getSimModeFromUser, this));
    for (const auto &warning : msr::airlib::AirSimSettings::singleton().warning_messages)
    {
        UAirBlueprintLib::LogMessageString(warning, "", LogDebugLevel::Failure);
    }
    for (const auto &error : msr::airlib::AirSimSettings::singleton().error_messages)
    {
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok, error, "settings.json");
    }
}


// Attempts to parse the settings text from one of multiple locations.
// First, check the command line for settings provided via "-s" or "--settings" arguments
// Next, check the default settings file location
// If the settings file cannot be read, throw an exception

bool AAirSimGameMode::getSettingsText(std::string &settingsText)
{
    return (getSettingsTextFromCommandLine(settingsText) || readSettingsTextFromFile(FString(common_utils::FileSystem::getConfigFilePath().c_str()), settingsText));
}

// Attempts to parse the settings text from the command line
// Looks for the flag "--settings". If it exists, settingsText will be set to the value.
// Example: AirSim.exe -s '{"foo" : "bar"}' -> settingsText will be set to {"foo": "bar"}
// Returns true if the argument is present, false otherwise.
bool AAirSimGameMode::getSettingsTextFromCommandLine(std::string &settingsText)
{

    bool found = false;
    FString settingsTextFString;
    const TCHAR *commandLineArgs = FCommandLine::Get();

    if (FParse::Param(commandLineArgs, TEXT("-settings")))
    {
        FString commandLineArgsFString = FString(commandLineArgs);
        int idx = commandLineArgsFString.Find(TEXT("-settings"));
        FString settingsJsonFString = commandLineArgsFString.RightChop(idx + 10);
        if (FParse::QuotedString(*settingsJsonFString, settingsTextFString))
        {
            settingsText = std::string(TCHAR_TO_UTF8(*settingsTextFString));
            found = true;
        }
    }

    return found;
}

bool AAirSimGameMode::readSettingsTextFromFile(FString settingsFilepath, std::string &settingsText)
{

    bool found = FPaths::FileExists(settingsFilepath);
    if (found)
    {
        FString settingsTextFStr;
        bool readSuccessful = FFileHelper::LoadFileToString(settingsTextFStr, *settingsFilepath);
        if (readSuccessful)
        {
            UAirBlueprintLib::LogMessageString("Loaded settings from ", TCHAR_TO_UTF8(*settingsFilepath), LogDebugLevel::Informational);
            settingsText = TCHAR_TO_UTF8(*settingsTextFStr);
        }
        else
        {
            UAirBlueprintLib::LogMessageString("Cannot read file ", TCHAR_TO_UTF8(*settingsFilepath), LogDebugLevel::Failure);
            throw std::runtime_error("Cannot read settings file.");
        }
    }

    return found;
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
