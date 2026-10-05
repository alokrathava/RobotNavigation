package com.alokrathava.sdk

import com.alokrathava.sdk.discovery.RobotDiscoveryManagerImpl
import com.alokrathava.sdk.internal.protocol.ROBOT_PROTOCOL_VERSION
import com.alokrathava.sdk.internal.serialization.ProtocolCodec
import com.alokrathava.sdk.model.RobotCapability
import java.io.File
import java.net.InetAddress
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

class RobotNavigationProtocolContractTest {
    private val fixtures = File("../../protocol-fixtures")

    private fun fixture(name: String): String {
        val file = File(fixtures, name)
        assertTrue("Fixture file must exist: ${file.absolutePath}", file.isFile)
        return file.readText()
    }

    @Test
    fun everyRobotNavigationFixtureDecodesAsProtocolV1() {
        val files = fixtures.listFiles { file -> file.extension == "json" }?.sortedBy { it.name }
            ?: error("Fixture directory is missing: ${fixtures.absolutePath}")
        assertTrue("Expected protocol fixtures", files.isNotEmpty())
        files.forEach { file ->
            val envelope = ProtocolCodec.decodeEnvelope(file.readText())
            assertTrue("${file.name} has an empty type", envelope.type.isNotBlank())
            assertEquals("${file.name} protocol version", ROBOT_PROTOCOL_VERSION, envelope.protocolVersion)
        }
    }

    @Test
    fun helloAckExposesCanonicalCapabilities() {
        val envelope = ProtocolCodec.decodeEnvelope(fixture("hello_ack.json"))
        val payload = ProtocolCodec.decodeHelloAck(envelope)
        assertNotNull(payload)
        assertEquals("0.2.0", payload?.gatewayVersion)
        val expected = listOf(
            "navigation", "manual_control", "docking", "safety", "telemetry",
            "battery", "health", "map_switching", "map_management", "mapping",
            "virtual_walls", "saved_locations", "waypoint_navigation", "missions",
            "robot_configuration", "robot_discovery", "diagnostics", "log_retrieval"
        )
        assertEquals(expected, payload?.capabilities)
        assertEquals(expected, RobotCapability.entries.map { it.name.lowercase() })
    }

    @Test
    fun stateFixturesUseTheSameMapStateDto() {
        val event = ProtocolCodec.decodeMapState(
            ProtocolCodec.decodeEnvelope(fixture("map_state.json")))
        val snapshot = ProtocolCodec.decodeRobotStateSnapshot(
            ProtocolCodec.decodeEnvelope(fixture("robot_state_snapshot.json")))
        assertNotNull(event)
        assertNotNull(snapshot?.mapState)
        assertEquals(event, snapshot?.mapState)
    }

    @Test
    fun telemetryAndDiscoveryFixturesDecode() {
        val telemetry = ProtocolCodec.decodeTelemetry(
            ProtocolCodec.decodeEnvelope(fixture("telemetry.json")))
        assertEquals("main_floor", telemetry?.activeMap)

        val discovery = RobotDiscoveryManagerImpl().parseBeacon(
            fixture("robot_discovery_beacon.json"), InetAddress.getByName("192.168.1.120"))
        assertNotNull(discovery)
        assertEquals("robot-01", discovery?.id)
        assertEquals(8080, discovery?.port)
    }

    @Test
    fun longRunningMapOperationsUseCommandAck() {
        val ack = ProtocolCodec.decodeEnvelope(fixture("command_ack.json"))
        assertEquals("command_ack", ack.type)
        assertFalse(File(fixtures, "save_map_ack.json").exists())
    }
}
