#include "config.h"
#include "imu.h"
#include "elrs.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

WebServer server(80);
WiFiServer server2(81);
WiFiClient client;
Preferences prefs;

volatile bool wifi_config_mode = false;
bool serverStarted = false;            

void wifiTask(void *pv)
{
    prefs.begin("pid", false); // "pid" is namespace
    prefs.end();
    while (true)
    {

        if (!wifi_config_mode && !armed && !serverStarted)
        {
            Serial.println("Starting Wi-Fi Config Portal...");
            loadPID(); // Load PID values from preferences
            startWiFiConfigPortal();
            delay(1000);          // Allow time for the server to start
            serverStarted = true; // Set server started flag
        }
        // else if (wifi_config_mode && armed)
        // {
        //     turnOffWiFi();
        //     serverStarted = false; // Reset server started flag
        // }

        server.handleClient(); // Handle incoming requests
        if (!client || !client.connected())
        {
            // Try to accept a new connection if no client is connected
            client = server2.available();
            if (client)
            {
                Serial.println("New client connected");
            }
            // return; // No data sent if no client is connected
        }
        else
        {
            // If client is connected, send PID values
            if (client.connected())
            {
                client.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                              AngleRoll, AnglePitch,
                              RateRoll, RatePitch, RateYaw,
                              PAnglePitch, IAnglePitch, DAnglePitch);
            }
            else
            {
                // If the client disconnected unexpectedly, reset it
                Serial.println("Client disconnected");
                client.stop();
            }
        }

        delay(10); // Prevent watchdog reset
    }
}

void startWiFiConfigPortal()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(wifi_ssid, wifi_password);
    Serial.println("Portal IP: " + WiFi.softAPIP().toString());
    server.on("/", handleRoot);
    server.on("/update", HTTP_POST, handleUpdate);
    server.begin();
    server2.begin(); 
    Serial.println("Server ready.");
    wifi_config_mode = true; // Enable Wi-Fi config mode
}

void handleRoot()
{
    String html = "<html><body><h2>Flight Controller PID Config</h2>";
    html += "<form action='/update' method='POST'>";

    html += "<h3>Roll PID</h3>";
    html += "Kp Roll: <input name='kp_roll' value='" + String(PRateRoll, 4) + "'><br>";
    html += "Ki Roll: <input name='ki_roll' value='" + String(IRateRoll, 4) + "'><br>";
    html += "Kd Roll: <input name='kd_roll' value='" + String(DRateRoll, 4) + "'><br>";
    // same for pitch
    html += "<h3>Pitch PID</h3>";
    html += "Kp Pitch: <input name='kp_pitch' value='" + String(PRatePitch, 4) + "'><br>";
    html += "Ki Pitch: <input name='ki_pitch' value='" + String(IRatePitch, 4) + "'><br>";
    html += "Kd Pitch: <input name='kd_pitch' value='" + String(DRatePitch, 4) + "'><br>";

    // PAngleRoll, PAnglePitch, PAngleYaw
    html += "<h3>Angle PID</h3>";
    html += "Kp Angle Roll: <input name='kp_angle_roll' value='" + String(PAngleRoll, 4) + "'><br>";
    html += "Ki Angle Roll: <input name='ki_angle_roll' value='" + String(IAngleRoll, 4) + "'><br>";
    html += "Kd Angle Roll: <input name='kd_angle_roll' value='" + String(DAngleRoll, 4) + "'><br>";
    html += "Kp Angle Pitch: <input name='kp_angle_pitch' value='" + String(PAnglePitch, 4) + "'><br>";
    html += "Ki Angle Pitch: <input name='ki_angle_pitch' value='" + String(IAnglePitch, 4) + "'><br>";
    html += "Kd Angle Pitch: <input name='kd_angle_pitch' value='" + String(DAnglePitch, 4) + "'><br>";

    html += "<h3>Yaw PID</h3>";
    html += "Kp Yaw: <input name='kp_yaw' value='" + String(PRateYaw, 4) + "'><br>";
    html += "Ki Yaw: <input name='ki_yaw' value='" + String(IRateYaw, 4) + "'><br>";
    html += "Kd Yaw: <input name='kd_yaw' value='" + String(DRateYaw, 4) + "'><br><br>";

    html += "<h3>Trim</h3>";
    html += "Trim Roll: <input name='trim_roll' value='" + String(trimRoll, 4) + "'><br>";
    html += "Trim Pitch: <input name='trim_pitch' value='" + String(trimPitch, 4) + "'><br>";

    html += "<input type='submit' value='Update PID Values'>";
    html += "</form><br><br>";

    html += "</body></html>";
    server.send(200, "text/html", html);
}

void handleUpdate()
{
    if (server.hasArg("kp_roll"))
    {
        PRateRoll = server.arg("kp_roll").toFloat();
    }
    if (server.hasArg("ki_roll"))
    {
        IRateRoll = server.arg("ki_roll").toFloat();
    }
    if (server.hasArg("kd_roll"))
    {
        DRateRoll = server.arg("kd_roll").toFloat();
    }
    if (server.hasArg("kp_pitch"))
    {
        PRatePitch = server.arg("kp_pitch").toFloat();
    }
    if (server.hasArg("ki_pitch"))
    {
        IRatePitch = server.arg("ki_pitch").toFloat();
    }
    if (server.hasArg("kd_pitch"))
    {
        DRatePitch = server.arg("kd_pitch").toFloat();
    }
    if (server.hasArg("kp_angle_roll"))
    {
        PAngleRoll = server.arg("kp_angle_roll").toFloat();
    }
    if (server.hasArg("ki_angle_roll"))
    {
        IAngleRoll = server.arg("ki_angle_roll").toFloat();
    }
    if (server.hasArg("kd_angle_roll"))
    {
        DAngleRoll = server.arg("kd_angle_roll").toFloat();
    }
    if (server.hasArg("kp_angle_pitch"))
    {
        PAnglePitch = server.arg("kp_angle_pitch").toFloat();
    }
    if (server.hasArg("ki_angle_pitch"))
    {
        IAnglePitch = server.arg("ki_angle_pitch").toFloat();
    }
    if (server.hasArg("kd_angle_pitch"))
    {
        DAnglePitch = server.arg("kd_angle_pitch").toFloat();
    }

    if (server.hasArg("kp_yaw"))
    {
        PRateYaw = server.arg("kp_yaw").toFloat();
    }
    if (server.hasArg("ki_yaw"))
    {
        IRateYaw = server.arg("ki_yaw").toFloat();
    }
    if (server.hasArg("kd_yaw"))
    {
        DRateYaw = server.arg("kd_yaw").toFloat();
    }

    if (server.hasArg("trim_roll"))
    {
        trimRoll = server.arg("trim_roll").toFloat();
    }
    if (server.hasArg("trim_pitch"))
    {
        trimPitch = server.arg("trim_pitch").toFloat();
    }
    savePID(); // Save updated PID values to preferences
    server.send(200, "text/html", "<h3>Settings Updated! <a href='/'>Back</a></h3>");
}

void savePID()
{
    if (prefs.begin("pid", false))
    {
        prefs.putFloat("kp_roll", PRateRoll);
        prefs.putFloat("ki_roll", IRateRoll);
        prefs.putFloat("kd_roll", DRateRoll);
        prefs.putFloat("kp_pitch", PRatePitch);
        prefs.putFloat("ki_pitch", IRatePitch);
        prefs.putFloat("kd_pitch", DRatePitch);

        prefs.putFloat("kp_angle_roll", PAngleRoll);
        prefs.putFloat("ki_angle_roll", IAngleRoll);
        prefs.putFloat("kd_angle_roll", DAngleRoll);
        prefs.putFloat("kp_angle_pitch", PAnglePitch);
        prefs.putFloat("ki_angle_pitch", IAnglePitch);
        prefs.putFloat("kd_angle_pitch", DAnglePitch);

        prefs.putFloat("kp_yaw", PRateYaw);
        prefs.putFloat("ki_yaw", IRateYaw);
        prefs.putFloat("kd_yaw", DRateYaw);

        prefs.putFloat("trim_roll", trimRoll);
        prefs.putFloat("trim_pitch", trimPitch);
        prefs.end();
    }
    else
    {
        Serial.println("Failed to save PID values to NVS.");
    }
}

void loadPID()
{
    if (prefs.begin("pid", true))
    {
        PRateRoll = prefs.getFloat("kp_roll", 1.0);
        IRateRoll = prefs.getFloat("ki_roll", 0.0);
        DRateRoll = prefs.getFloat("kd_roll", 0.0);
        PRateYaw = prefs.getFloat("kp_yaw", 1.0);
        IRateYaw = prefs.getFloat("ki_yaw", 0.0);
        DRateYaw = prefs.getFloat("kd_yaw", 0.0);

        PAngleRoll = prefs.getFloat("kp_angle_roll", 1.0);
        IAngleRoll = prefs.getFloat("ki_angle_roll", 0.0);
        DAngleRoll = prefs.getFloat("kd_angle_roll", 0.0);
        PAnglePitch = prefs.getFloat("kp_angle_pitch", 1.0);
        IAnglePitch = prefs.getFloat("ki_angle_pitch", 0.0);
        DAnglePitch = prefs.getFloat("kd_angle_pitch", 0.0);

        PRatePitch = prefs.getFloat("kp_pitch", 1.0);
        IRatePitch = prefs.getFloat("ki_pitch", 0.0);
        DRatePitch = prefs.getFloat("kd_pitch", 0.0);

        trimRoll = prefs.getFloat("trim_roll", 0.0);
        trimPitch = prefs.getFloat("trim_pitch", 0.0);
        
        prefs.end();
    }
    else
    {
        Serial.println("Failed to load PID values from NVS.");
    }
}

void turnOffWiFi()
{
    WiFi.disconnect(true);    // Disconnect and erase stored credentials
    WiFi.mode(WIFI_OFF);      // Turn off Wi-Fi hardware
    wifi_config_mode = false; // Disable Wi-Fi config mode
}

void testMotor(int speed, int motor)
{
    if (motor == 1)
        mot1.writeMicroseconds(speed); // Set motor 1 speed
    else if (motor == 2)
        mot2.writeMicroseconds(speed); // Set motor 2 speed
    else if (motor == 3)
        mot3.writeMicroseconds(speed); // Set motor 3 speed
    else if (motor == 4)
        mot4.writeMicroseconds(speed); // Set motor 4 speed
    delay(1000);
}

void stopMotor(int motor)
{
    if (motor == 1)
        mot1.writeMicroseconds(1000); // Stop motor 1
    else if (motor == 2)
        mot2.writeMicroseconds(1000); // Stop motor 2
    else if (motor == 3)
        mot3.writeMicroseconds(1000); // Stop motor 3
    else if (motor == 4)
        mot4.writeMicroseconds(1000); // Stop motor 4
}