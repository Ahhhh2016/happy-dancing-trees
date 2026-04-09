/*!

 Settings.h
 CS123 Support Code

 @author  Evan Wallace (edwallac)
 @date    9/1/2010

 This file contains various settings and enumerations that you will need to
 use in the various assignments. The settings are bound to the GUI via static
 data bindings.

**/

#include "settings.h"
#include <QSettings>

Settings settings;

/**
 * @brief Loads the application settings
 */
void Settings::loadSettingsOrDefaults() {
    QSettings s("CS123", "CS123");
    imagePath = s.value("imagePath", "").toString();
}

/**
 * @brief Saves settings from this session to be loaded
 * in for next session.
 */
void Settings::saveSettings() {
    QSettings s("CS123", "CS123");
    s.setValue("imagePath", imagePath);
}
