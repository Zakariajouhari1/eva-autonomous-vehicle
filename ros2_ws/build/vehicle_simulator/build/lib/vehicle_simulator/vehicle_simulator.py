// Main.qml
// Interface HMI complète avec carte interactive et dashboard (Qt 6 compatible)

import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtLocation 6.2
import QtPositioning 6.2

Window {
    id: mainWindow
    width: 1280
    height: 720
    visible: true
    title: "EVA COCKPIT"
    color: "#000000"
    
    // État de l'interface
    property bool showMap: true
    property bool showDashboard: true
    
    // ======================================
    // ARRIÈRE-PLAN PRINCIPAL
    // ======================================
    Rectangle {
        anchors.fill: parent
        color: "#000000"
    }
    
    // ======================================
    // CARTE OPENSTREETMAP
    // ======================================
    Plugin {
        id: mapPlugin
        name: "osm"
        PluginParameter {
            name: "osm.mapping.providersrepository.disabled"
            value: "true"
        }
        PluginParameter {
            name: "osm.mapping.providersrepository.address"
            value: "http://maps-redirect.qt.io/osm/5.6/"
        }
    }
    
    Map {
        id: map
        anchors {
            top: topBar.bottom
            right: parent.right
            bottom: parent.bottom
            margins: 10
        }
        width: showMap ? parent.width * 0.45 : 0
        visible: showMap
        
        plugin: mapPlugin
        center: QtPositioning.coordinate(33.5731, -7.5898) // Casablanca
        zoomLevel: 14
        
        // Qt 6: Plus de gesture.enabled, la carte est interactive par défaut
        
        // ===== AFFICHAGE DE LA ROUTE =====
        
        // Route globale (ligne bleue OSRM)
        MapPolyline {
            id: globalRouteLine
            line.width: 6
            line.color: "#4285F4"
            visible: routeService && routeService.globalPath && routeService.globalPath.length > 0
            
            path: {
                if (!routeService || !routeService.globalPath) return [];
                
                var coords = [];
                var metersPerDegreeLat = 111320.0;
                var metersPerDegreeLon = 111320.0 * Math.cos(33.5731 * Math.PI / 180.0);
                
                for (var i = 0; i < routeService.globalPath.length; i++) {
                    var pt = routeService.globalPath[i];
                    var lat = 33.5731 + (pt.y / metersPerDegreeLat);
                    var lon = -7.5898 + (pt.x / metersPerDegreeLon);
                    coords.push(QtPositioning.coordinate(lat, lon));
                }
                return coords;
            }
        }
        
        // Marqueur de départ (A)
        MapQuickItem {
            visible: routeService && routeService.globalPath && routeService.globalPath.length > 0
            
            coordinate: {
                if (!routeService || !routeService.globalPath || routeService.globalPath.length === 0) {
                    return QtPositioning.coordinate(33.5731, -7.5898);
                }
                var pt = routeService.globalPath[0];
                var metersPerDegreeLat = 111320.0;
                var metersPerDegreeLon = 111320.0 * Math.cos(33.5731 * Math.PI / 180.0);
                var lat = 33.5731 + (pt.y / metersPerDegreeLat);
                var lon = -7.5898 + (pt.x / metersPerDegreeLon);
                return QtPositioning.coordinate(lat, lon);
            }
            
            anchorPoint: Qt.point(15, 30)
            
            sourceItem: Rectangle {
                width: 30
                height: 30
                radius: 15
                color: "#4285F4"
                border.color: "white"
                border.width: 3
                
                Text {
                    anchors.centerIn: parent
                    text: "A"
                    color: "white"
                    font.bold: true
                    font.pixelSize: 16
                }
            }
        }
        
        // Marqueur d'arrivée (B)
        MapQuickItem {
            visible: routeService && routeService.globalPath && routeService.globalPath.length > 0
            
            coordinate: {
                if (!routeService || !routeService.globalPath || routeService.globalPath.length === 0) {
                    return QtPositioning.coordinate(33.5731, -7.5898);
                }
                var pt = routeService.globalPath[routeService.globalPath.length - 1];
                var metersPerDegreeLat = 111320.0;
                var metersPerDegreeLon = 111320.0 * Math.cos(33.5731 * Math.PI / 180.0);
                var lat = 33.5731 + (pt.y / metersPerDegreeLat);
                var lon = -7.5898 + (pt.x / metersPerDegreeLon);
                return QtPositioning.coordinate(lat, lon);
            }
            
            anchorPoint: Qt.point(15, 30)
            
            sourceItem: Rectangle {
                width: 30
                height: 30
                radius: 15
                color: "#EA4335"
                border.color: "white"
                border.width: 3
                
                Text {
                    anchors.centerIn: parent
                    text: "B"
                    color: "white"
                    font.bold: true
                    font.pixelSize: 16
                }
            }
        }
        
        // Position actuelle du véhicule (point vert)
        MapQuickItem {
            coordinate: {
                if (!routeService) return QtPositioning.coordinate(33.5731, -7.5898);
                var metersPerDegreeLat = 111320.0;
                var metersPerDegreeLon = 111320.0 * Math.cos(33.5731 * Math.PI / 180.0);
                var lat = 33.5731 + (routeService.currentY / metersPerDegreeLat);
                var lon = -7.5898 + (routeService.currentX / metersPerDegreeLon);
                return QtPositioning.coordinate(lat, lon);
            }
            
            anchorPoint: Qt.point(20, 20)
            
            sourceItem: Rectangle {
                width: 40
                height: 40
                radius: 20
                color: "#34A853"
                border.color: "white"
                border.width: 4
                
                Rectangle {
                    anchors.centerIn: parent
                    width: 8
                    height: 8
                    radius: 4
                    color: "white"
                }
            }
        }
        
        // Clic sur la carte pour définir destination
        MouseArea {
            anchors.fill: parent
            onClicked: function(mouse) {
                var coord = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                console.log("Carte cliquée:", coord.latitude, coord.longitude)
                
                // Convertir GPS vers coordonnées locales
                var metersPerDegreeLat = 111320.0;
                var metersPerDegreeLon = 111320.0 * Math.cos(33.5731 * Math.PI / 180.0);
                var x = (coord.longitude - (-7.5898)) * metersPerDegreeLon;
                var y = (coord.latitude - 33.5731) * metersPerDegreeLat;
                
                console.log("Local:", x, y)
                
                // Envoyer le goal à ROS2
                routeService.navigateTo(x, y)
            }
        }
        
        // Boutons de contrôle de la carte
        Column {
            anchors {
                right: parent.right
                top: parent.top
                margins: 10
            }
            spacing: 5
            z: 100
            
            // Zoom +
            Rectangle {
                width: 40
                height: 40
                radius: 20
                color: "#4285F4"
                
                Text {
                    anchors.centerIn: parent
                    text: "+"
                    font.pixelSize: 24
                    font.bold: true
                    color: "white"
                }
                
                MouseArea {
                    anchors.fill: parent
                    onClicked: map.zoomLevel += 1
                }
            }
            
            // Zoom -
            Rectangle {
                width: 40
                height: 40
                radius: 20
                color: "#4285F4"
                
                Text {
                    anchors.centerIn: parent
                    text: "−"
                    font.pixelSize: 24
                    font.bold: true
                    color: "white"
                }
                
                MouseArea {
                    anchors.fill: parent
                    onClicked: map.zoomLevel -= 1
                }
            }
            
            // Centrer sur position
            Rectangle {
                width: 40
                height: 40
                radius: 20
                color: "#34A853"
                
                Text {
                    anchors.centerIn: parent
                    text: "⌖"
                    font.pixelSize: 24
                    color: "white"
                }
                
                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        map.center = QtPositioning.coordinate(33.5731, -7.5898)
                        map.zoomLevel = 14
                    }
                }
            }
        }
    }
    
    // ======================================
    // DASHBOARD GAUCHE
    // ======================================
    Flickable {
        id: dashboardArea
        anchors {
            top: topBar.bottom
            left: parent.left
            bottom: parent.bottom
            right: showMap ? map.left : parent.right
            margins: 10
        }
        contentHeight: dashboardColumn.height
        clip: true
        
        Column {
            id: dashboardColumn
            width: parent.width
            spacing: 20
            padding: 20
            
            
            // État du véhicule
            VehicleStatusPanel {
                width: parent.width - 40
                anchors.horizontalCenter: parent.horizontalCenter
            }
            
            // Speedomètre
            Rectangle {
                width: 200
                height: 200
                radius: 100
                color: "#2A2A2A"
                border.color: "#4285F4"
                border.width: 4
                
                Column {
                    anchors.centerIn: parent
                    spacing: 10
                    
                    Text {
                        text: Math.round(routeService ? routeService.currentSpeed : 0)
                        color: "white"
                        font.pixelSize: 72
                        font.bold: true
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    
                    Text {
                        text: "km/h"
                        color: "#888"
                        font.pixelSize: 18
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    
                    Text {
                        text: "Current Speed"
                        color: "#4285F4"
                        font.pixelSize: 14
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }
            }
            
            // Informations de navigation
            Rectangle {
                width: 350
                height: 200
                color: "#2A2A2A"
                radius: 12
                visible: routeService && routeService.isNavigating
                
                Column {
                    anchors.fill: parent
                    anchors.margins: 20
                    spacing: 15
                    
                    Text {
                        text: "🧭 Navigation Active"
                        color: "#4285F4"
                        font.pixelSize: 18
                        font.bold: true
                    }
                    
                    Row {
                        spacing: 30
                        width: parent.width
                        
                        // Distance restante
                        Column {
                            spacing: 5
                            Rectangle {
                                width: 80
                                height: 60
                                color: "#1A1A1A"
                                radius: 8
                                
                                Column {
                                    anchors.centerIn: parent
                                    Text {
                                        text: routeService ? routeService.remainingDistance.toFixed(1) : "0.0"
                                        color: "#34A853"
                                        font.pixelSize: 20
                                        font.bold: true
                                        anchors.horizontalCenter: parent.horizontalCenter
                                    }
                                    Text {
                                        text: "km left"
                                        color: "#888"
                                        font.pixelSize: 11
                                        anchors.horizontalCenter: parent.horizontalCenter
                                    }
                                }
                            }
                        }
                        
                        // Temps restant
                        Column {
                            spacing: 5
                            Rectangle {
                                width: 80
                                height: 60
                                color: "#1A1A1A"
                                radius: 8
                                
                                Column {
                                    anchors.centerIn: parent
                                    Text {
                                        text: routeService ? Math.round(routeService.remainingTime) : "0"
                                        color: "#FBBC04"
                                        font.pixelSize: 20
                                        font.bold: true
                                        anchors.horizontalCenter: parent.horizontalCenter
                                    }
                                    Text {
                                        text: "min"
                                        color: "#888"
                                        font.pixelSize: 11
                                        anchors.horizontalCenter: parent.horizontalCenter
                                    }
                                }
                            }
                        }
                        
                        // Progression
                        Column {
                            spacing: 5
                            Rectangle {
                                width: 80
                                height: 60
                                color: "#1A1A1A"
                                radius: 8
                                
                                Column {
                                    anchors.centerIn: parent
                                    Text {
                                        text: routeService ? routeService.progressPercent : "0"
                                        color: "#EA4335"
                                        font.pixelSize: 20
                                        font.bold: true
                                        anchors.horizontalCenter: parent.horizontalCenter
                                    }
                                    Text {
                                        text: "% done"
                                        color: "#888"
                                        font.pixelSize: 11
                                        anchors.horizontalCenter: parent.horizontalCenter
                                    }
                                }
                            }
                        }
                    }
                    
                    // Instruction
                    Rectangle {
                        width: parent.width
                        height: 50
                        color: "#1A1A1A"
                        radius: 8
                        
                        Text {
                            anchors.centerIn: parent
                            text: routeService ? routeService.nextInstruction : "No navigation"
                            color: "white"
                            font.pixelSize: 14
                            elide: Text.ElideRight
                            width: parent.width - 20
                            horizontalAlignment: Text.AlignHCenter
                        }
                    }
                }
            }
            
            // Informations route (mode statique)
            Rectangle {
                width: 350
                height: 140
                color: "#2A2A2A"
                radius: 12
                visible: routeService && routeService.waypointCount > 0 && !routeService.isNavigating
                
                Column {
                    anchors.fill: parent
                    anchors.margins: 20
                    spacing: 15
                    
                    Text {
                        text: "📍 Route calculée"
                        color: "#4285F4"
                        font.pixelSize: 18
                        font.bold: true
                    }
                    
                    Row {
                        spacing: 20
                        width: parent.width
                        
                        Column {
                            spacing: 5
                            Text {
                                text: routeService ? routeService.routeDistance.toFixed(2) + " km" : "0 km"
                                color: "#34A853"
                                font.pixelSize: 16
                                font.bold: true
                            }
                            Text {
                                text: "Distance"
                                color: "#888"
                                font.pixelSize: 12
                            }
                        }
                        
                        Column {
                            spacing: 5
                            Text {
                                text: routeService ? Math.round(routeService.routeDuration) + " min" : "0 min"
                                color: "#FBBC04"
                                font.pixelSize: 16
                                font.bold: true
                            }
                            Text {
                                text: "Durée"
                                color: "#888"
                                font.pixelSize: 12
                            }
                        }
                        
                        Column {
                            spacing: 5
                            Text {
                                text: routeService ? routeService.waypointCount + " pts" : "0 pts"
                                color: "#EA4335"
                                font.pixelSize: 16
                                font.bold: true
                            }
                            Text {
                                text: "Waypoints"
                                color: "#888"
                                font.pixelSize: 12
                            }
                        }
                    }
                }
            }
        }
    }
    
    // ======================================
    // BARRE SUPÉRIEURE
    // ======================================
    Rectangle {
        id: topBar
        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
        }
        height: 60
        color: "#1A1A1A"
        
        Row {
            anchors.verticalCenter: parent.verticalCenter
            anchors.left: parent.left
            anchors.leftMargin: 20
            spacing: 15
            
            Text {
                text: "EVA_COCKPIT"
                color: "#4285F4"
                font.pixelSize: 24
                font.bold: true
            }
            
            Text {
                text: "|"
                color: "#555"
                font.pixelSize: 24
            }
            
            Text {
                text: Qt.formatDateTime(new Date(), "MMM dd  hh:mm AP")
                color: "white"
                font.pixelSize: 16
            }
        }
        
        Row {
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 20
            spacing: 20
            
            // Indicateur ROS2
            Rectangle {
                width: 100
                height: 35
                radius: 17.5
                color: routeService && routeService.waypointCount > 0 ? "#34A853" : "#555"
                
                Text {
                    anchors.centerIn: parent
                    text: routeService && routeService.waypointCount > 0 ? "🟢 ROS2" : "⚪ ROS2"
                    color: "white"
                    font.pixelSize: 14
                }
            }
            
            // Toggle Map
            Rectangle {
                width: 120
                height: 35
                radius: 17.5
                color: showMap ? "#4285F4" : "#555"
                
                Text {
                    anchors.centerIn: parent
                    text: showMap ? "🗺️ Map ON" : "🗺️ Map OFF"
                    color: "white"
                    font.pixelSize: 14
                }
                
                MouseArea {
                    anchors.fill: parent
                    onClicked: showMap = !showMap
                }
            }
            
            // Température
            Text {
                text: "🌡️ 17°C"
                color: "white"
                font.pixelSize: 16
                anchors.verticalCenter: parent.verticalCenter
            }
            
            // Bluetooth
            Text {
                text: "📶"
                color: "#4285F4"
                font.pixelSize: 20
                anchors.verticalCenter: parent.verticalCenter
            }
            
            // WiFi
            Text {
                text: "📡"
                color: "#4285F4"
                font.pixelSize: 20
                anchors.verticalCenter: parent.verticalCenter
            }
        }
    }
    
    // ======================================
    // PANNEAU D'INFORMATIONS DE ROUTE (overlay)
    // ======================================
    RouteInfoPanel {
        id: routeInfoPanel
        anchors {
            bottom: parent.bottom
            horizontalCenter: parent.horizontalCenter
            bottomMargin: 20
        }
        routeService: routeService
        z: 100
    }
    
    // ======================================
    // NOTIFICATIONS
    // ======================================
    Rectangle {
        id: notification
        anchors {
            top: topBar.bottom
            horizontalCenter: parent.horizontalCenter
            topMargin: 20
        }
        width: 400
        height: 60
        radius: 10
        color: "#2A2A2A"
        border.color: "#4285F4"
        border.width: 2
        visible: false
        z: 200
        
        property string message: ""
        
        Text {
            anchors.centerIn: parent
            text: notification.message
            color: "white"
            font.pixelSize: 16
        }
        
        Timer {
            id: notificationTimer
            interval: 3000
            onTriggered: notification.visible = false
        }
        
        function show(msg) {
            notification.message = msg
            notification.visible = true
            notificationTimer.restart()
        }
    }
    
    // Connexions aux événements ROS2
    Connections {
        target: routeService
        
        function onRouteReady() {
            console.log("Route prête!")
            notification.show("✓ Route calculée avec succès")
        }
        
        function onNavigationStarted() {
            console.log("Navigation démarrée")
            notification.show("🧭 Navigation démarrée")
        }
        
        function onNavigationCompleted() {
            console.log("Destination atteinte!")
            notification.show("🎉 Vous êtes arrivé!")
        }
    }
}
