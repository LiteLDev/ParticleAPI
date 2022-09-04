/**
 * @file Plugin.cpp
 * @brief The main file of the plugin
 */

#include <LoggerAPI.h>
#include <EventAPI.h>
#include <MC/ResourcePackRepository.hpp>
/**
 * @brief The entrypoint of the plugin. DO NOT remove or rename this function.
 *        
 */


void PluginInit()
{
    Event::ResourcePackInitEvent::subscribe([](const Event::ResourcePackInitEvent& ev) {
        ev.mRepo->setCustomResourcePackPath(PackType::PackType_Resources,".\\plugins\\LiteLoader\\ResourcePacks");
        return true;
    });
}
