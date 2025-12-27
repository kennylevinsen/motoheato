import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart' as fbp;
import 'package:permission_handler/permission_handler.dart';
import 'package:flutter_svg/flutter_svg.dart';

void main() {
  runApp(const MotoheatoApp());
}

class MotoheatoApp extends StatelessWidget {
  const MotoheatoApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Motoheato',
      theme: ThemeData(
        brightness: Brightness.dark,
        primarySwatch: Colors.orange,
        useMaterial3: true,
      ),
      home: const ConfigPage(),
    );
  }
}

class MonitoringData {
  final int state;
  final bool oilPressureOk;
  final double voltage;
  final double temperature;
  final int dutyL;
  final int dutyR;
  final int dutyS;

  MonitoringData({
    required this.state,
    required this.oilPressureOk,
    required this.voltage,
    required this.temperature,
    required this.dutyL,
    required this.dutyR,
    required this.dutyS,
  });

  factory MonitoringData.empty() {
    return MonitoringData(
      state: 0xFF,
      oilPressureOk: false,
      voltage: 0.0,
      temperature: 0.0,
      dutyL: 0,
      dutyR: 0,
      dutyS: 0,
    );
  }

  factory MonitoringData.fromBytes(List<int> bytes) {
    int offset = 0;
    if (bytes.length >= 13) offset = 2; // Skip Company ID

    final data = ByteData.sublistView(Uint8List.fromList(bytes), offset);
    return MonitoringData(
      state: data.getUint8(0),
      oilPressureOk: data.getUint8(1) != 0,
      voltage: data.getUint16(2, Endian.little) / 100.0,
      temperature: data.getInt16(4, Endian.little) / 100.0,
      dutyL: data.getUint16(6, Endian.little),
      dutyR: data.getUint16(8, Endian.little),
      dutyS: data.getUint16(10, Endian.little),
    );
  }
}

class ConfigPage extends StatefulWidget {
  const ConfigPage({super.key});

  @override
  State<ConfigPage> createState() => _ConfigPageState();
}

class _ConfigPageState extends State<ConfigPage> {
  fbp.BluetoothDevice? _device;
  fbp.BluetoothCharacteristic? _configChar;
  fbp.BluetoothCharacteristic? _monitorChar;
  bool _isConnecting = false;
  bool _isLoading = false;
  MonitoringData? _monitoringData;

  StreamSubscription? _scanSubscription;
  StreamSubscription? _monitorSubscription;
  StreamSubscription? _connectionSubscription;

  // Config State
  double _voltageThreshold = 13.5;
  double _voltageHysteresis = 0.5;
  int _startDelaySeconds = 10;
  double _gripLeftFactor = 1.0;
  double _gripRightFactor = 0.9;
  double _seatFactor = 1.0;

  double _tempSetpoint = 15.0;
  double _tempPGain = 5.0;
  double _tempMaxPower = 1.0;
  double _preheatGain = 20.0;
  int _preheatMaxTime = 300;

  bool _oilPressureEnabled = true;

  final String serviceUuid = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
  final String configUuid = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
  final String monitorUuid = "beb5483e-36e1-4688-b7f5-ea07361b26a9";

  @override
  void initState() {
    super.initState();
    _initApp();
  }

  @override
  void dispose() {
    _cleanupConnection();
    _scanSubscription?.cancel();
    super.dispose();
  }

  void _cleanupConnection() {
    _monitorSubscription?.cancel();
    _connectionSubscription?.cancel();
    _monitorSubscription = null;
    _connectionSubscription = null;
    _device = null;
    _configChar = null;
    _monitorChar = null;
  }

  void _initApp() async {
    await _requestPermissions();
    _startScan();
  }

  Future<void> _requestPermissions() async {
    await [
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
      Permission.location,
    ].request();
  }

  void _startScan() async {
    if (_isConnecting || _device != null) return;

    _scanSubscription?.cancel();
    _scanSubscription = fbp.FlutterBluePlus.onScanResults.listen((results) {
      for (fbp.ScanResult r in results) {
        String name = r.device.platformName.isEmpty
            ? r.advertisementData.advName
            : r.device.platformName;
        if (name == "Motoheato" && _device == null && !_isConnecting)
          _connect(r.device);
      }
    });

    try {
      await fbp.FlutterBluePlus.startScan(
        timeout: const Duration(minutes: 5),
        continuousUpdates: true,
      );
    } catch (e) {
      debugPrint("Scan error: $e");
    }
  }

  Future<void> _connect(fbp.BluetoothDevice device) async {
    setState(() => _isConnecting = true);
    await fbp.FlutterBluePlus.stopScan();

    try {
      await device
          .connect(license: fbp.License.free)
          .timeout(const Duration(seconds: 10));

      _connectionSubscription?.cancel();
      _connectionSubscription = device.connectionState.listen((state) {
        if (state == fbp.BluetoothConnectionState.disconnected) {
          if (mounted) {
            setState(() {
              _cleanupConnection();
              _startScan();
            });
          }
        }
      });

      setState(() => _device = device);

      List<fbp.BluetoothService> services = await device.discoverServices();
      for (var s in services) {
        if (s.uuid.toString().toLowerCase() == serviceUuid.toLowerCase()) {
          for (var c in s.characteristics) {
            if (c.uuid.toString().toLowerCase() == configUuid.toLowerCase()) {
              _configChar = c;
            } else if (c.uuid.toString().toLowerCase() ==
                monitorUuid.toLowerCase()) {
              _monitorChar = c;
              await _monitorChar!.setNotifyValue(true);
              _monitorSubscription?.cancel();
              _monitorSubscription = _monitorChar!.lastValueStream.listen((
                value,
              ) {
                if (value.isNotEmpty && mounted) {
                  setState(
                    () => _monitoringData = MonitoringData.fromBytes(value),
                  );
                }
              });
            }
          }
        }
      }
      _readConfig();
    } catch (e) {
      debugPrint("Connection error: $e");
      _startScan();
    } finally {
      if (mounted) setState(() => _isConnecting = false);
    }
  }

  int _calculateCrc(Uint8List data) {
    int crc = 0xFFFFFFFF;
    for (int byte in data) {
      crc ^= byte;
      for (int i = 0; i < 8; i++) {
        if ((crc & 1) != 0) {
          crc = (crc >> 1) ^ 0xEDB88320;
        } else {
          crc >>= 1;
        }
      }
    }
    return crc ^ 0xFFFFFFFF;
  }

  void _readConfig() async {
    if (_configChar == null) return;
    setState(() => _isLoading = true);

    try {
      List<int> value = await _configChar!.read();
      // Struct size: 11 fields * 4 bytes + 1 * 4 bytes (flags) + CRC(4) = 52 bytes
      if (value.length >= 48) {
        final byteData = ByteData.sublistView(Uint8List.fromList(value));
        setState(() {
          _voltageThreshold = byteData.getFloat32(0, Endian.little);
          _voltageHysteresis = byteData.getFloat32(4, Endian.little);
          _startDelaySeconds = byteData.getInt32(8, Endian.little);
          _gripLeftFactor = byteData.getFloat32(12, Endian.little);
          _gripRightFactor = byteData.getFloat32(16, Endian.little);
          _seatFactor = byteData.getFloat32(20, Endian.little);
          _tempSetpoint = byteData.getFloat32(24, Endian.little);
          _tempPGain = byteData.getFloat32(28, Endian.little);
          _tempMaxPower = byteData.getFloat32(32, Endian.little);
          _preheatGain = byteData.getFloat32(36, Endian.little);
          _preheatMaxTime = byteData.getInt32(40, Endian.little);
          _oilPressureEnabled = byteData.getUint8(44) != 0;
        });
      }
    } catch (e) {
      debugPrint("Read error: $e");
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  void _saveConfig() async {
    if (_configChar == null) return;
    setState(() => _isLoading = true);

    final byteData = ByteData(52);
    byteData.setFloat32(0, _voltageThreshold, Endian.little);
    byteData.setFloat32(4, _voltageHysteresis, Endian.little);
    byteData.setInt32(8, _startDelaySeconds, Endian.little);
    byteData.setFloat32(12, _gripLeftFactor, Endian.little);
    byteData.setFloat32(16, _gripRightFactor, Endian.little);
    byteData.setFloat32(20, _seatFactor, Endian.little);
    byteData.setFloat32(24, _tempSetpoint, Endian.little);
    byteData.setFloat32(28, _tempPGain, Endian.little);
    byteData.setFloat32(32, _tempMaxPower, Endian.little);
    byteData.setFloat32(36, _preheatGain, Endian.little);
    byteData.setInt32(40, _preheatMaxTime, Endian.little);
    byteData.setUint8(44, _oilPressureEnabled ? 1 : 0);
    // bytes 45, 46, 47 are padding (reserved)

    final crc = _calculateCrc(byteData.buffer.asUint8List(0, 48));
    byteData.setUint32(48, crc, Endian.little);

    try {
      await _configChar!.write(byteData.buffer.asUint8List());
      if (mounted) {
        ScaffoldMessenger.of(
          context,
        ).showSnackBar(const SnackBar(content: Text("Config Saved!")));
      }
    } catch (e) {
      debugPrint("Write error: $e");
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Motoheato'),
        actions: [
          if (_device != null)
            IconButton(icon: const Icon(Icons.refresh), onPressed: _readConfig),
        ],
      ),
      body: Column(
        children: [
          _buildMonitoringPanel(),
          Expanded(
            child: _isConnecting && _device == null
                ? Center(
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        _buildBigLogo(),
                        const SizedBox(height: 40),
                        const CircularProgressIndicator(),
                        const SizedBox(height: 20),
                        const Text("Connecting..."),
                      ],
                    ),
                  )
                : _device == null
                ? Center(
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        _buildBigLogo(),
                        const SizedBox(height: 40),
                        ElevatedButton(
                          style: ElevatedButton.styleFrom(
                            minimumSize: const Size(200, 50),
                            backgroundColor: Colors.orange,
                            foregroundColor: Colors.black,
                          ),
                          onPressed: _startScan,
                          child: const Text(
                            "Scan for Device",
                            style: TextStyle(fontWeight: FontWeight.bold),
                          ),
                        ),
                        if (_isConnecting)
                          const Padding(
                            padding: EdgeInsets.only(top: 16),
                            child: Text("Scanning..."),
                          ),
                      ],
                    ),
                  )
                : _isLoading
                ? const Center(child: CircularProgressIndicator())
                : ListView(
                    padding: const EdgeInsets.all(16),
                    children: [
                      _buildSection("Voltage & Startup"),
                      SwitchListTile(
                        title: const Text(
                          "Oil Pressure Monitoring",
                          style: TextStyle(fontSize: 14),
                        ),
                        value: _oilPressureEnabled,
                        onChanged: (v) =>
                            setState(() => _oilPressureEnabled = v),
                        activeColor: Colors.orange,
                      ),
                      Row(
                        children: [
                          Expanded(
                            child: _buildMiniSlider(
                              "THRESHOLD (V)",
                              _voltageThreshold,
                              10.0,
                              15.0,
                              (v) => setState(() => _voltageThreshold = v),
                              divisions: 50,
                            ),
                          ),
                          Expanded(
                            child: _buildMiniSlider(
                              "HYSTERESIS (V)",
                              _voltageHysteresis,
                              0.1,
                              2.0,
                              (v) => setState(() => _voltageHysteresis = v),
                              divisions: 19,
                            ),
                          ),
                        ],
                      ),
                      _buildIntInput(
                        "Start Delay (s)",
                        _startDelaySeconds,
                        (v) => setState(() => _startDelaySeconds = v),
                      ),

                      _buildSection("Temperature Curve"),
                      _buildCurvePreview("Expected Power", [20, 15, 10, 5, 0], (
                        t,
                      ) {
                        double error = _tempSetpoint - t;
                        double power = error * (_tempPGain / 100.0);
                        return "${(power.clamp(0.0, _tempMaxPower) * 100).round()}%";
                      }),
                      _buildMiniSlider(
                        "SETPOINT (°C)",
                        _tempSetpoint,
                        5.0,
                        40.0,
                        (v) => setState(() => _tempSetpoint = v),
                        divisions: 350,
                      ),
                      const Divider(),
                      Row(
                        children: [
                          Expanded(
                            child: _buildMiniSlider(
                              "P-GAIN",
                              _tempPGain,
                              1.0,
                              20.0,
                              (v) => setState(() => _tempPGain = v),
                              divisions: 19,
                            ),
                          ),
                          Expanded(
                            child: _buildMiniSlider(
                              "MAX POWER",
                              _tempMaxPower,
                              0.1,
                              1.0,
                              (v) => setState(() => _tempMaxPower = v),
                              divisions: 18,
                            ),
                          ),
                        ],
                      ),

                      _buildSection("Preheat"),
                      _buildCurvePreview(
                        "Expected Preheat",
                        [20, 15, 10, 5, 0],
                        (t) {
                          double error = _tempSetpoint - t;
                          double time = error * _preheatGain;
                          return "${time.clamp(0, _preheatMaxTime).round()}s";
                        },
                      ),
                      Row(
                        children: [
                          Expanded(
                            child: _buildMiniSlider(
                              "GAIN (s/°C)",
                              _preheatGain,
                              1.0,
                              60.0,
                              (v) => setState(() => _preheatGain = v),
                              divisions: 59,
                            ),
                          ),
                          Expanded(
                            child: _buildIntInput(
                              "MAX (s)",
                              _preheatMaxTime,
                              (v) => setState(() => _preheatMaxTime = v),
                            ),
                          ),
                        ],
                      ),

                      _buildSection("Power Factors"),
                      Row(
                        children: [
                          Expanded(
                            child: _buildMiniSlider(
                              "LEFT",
                              _gripLeftFactor,
                              0.0,
                              1.0,
                              (v) => setState(() => _gripLeftFactor = v),
                            ),
                          ),
                          Expanded(
                            child: _buildMiniSlider(
                              "SEAT",
                              _seatFactor,
                              0.0,
                              1.0,
                              (v) => setState(() => _seatFactor = v),
                            ),
                          ),
                          Expanded(
                            child: _buildMiniSlider(
                              "RIGHT",
                              _gripRightFactor,
                              0.0,
                              1.0,
                              (v) => setState(() => _gripRightFactor = v),
                            ),
                          ),
                        ],
                      ),

                      const SizedBox(height: 32),
                      ElevatedButton(
                        style: ElevatedButton.styleFrom(
                          backgroundColor: Colors.orange,
                          foregroundColor: Colors.black,
                          minimumSize: const Size.fromHeight(50),
                        ),
                        onPressed: _saveConfig,
                        child: const Text(
                          "SAVE TO DEVICE",
                          style: TextStyle(fontWeight: FontWeight.bold),
                        ),
                      ),
                    ],
                  ),
          ),
        ],
      ),
    );
  }

  Widget _buildBigLogo() =>
      SvgPicture.asset('assets/icon.svg', width: 200, height: 200);

  Widget _buildCurvePreview(
    String label,
    List<double> temps,
    String Function(double) calc,
  ) {
    return Container(
      margin: const EdgeInsets.only(bottom: 8),
      padding: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.black26,
        borderRadius: BorderRadius.circular(4),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: temps
            .map(
              (t) => Column(
                children: [
                  Text(
                    "${t.round()}°C",
                    style: const TextStyle(fontSize: 9, color: Colors.grey),
                  ),
                  Text(
                    calc(t),
                    style: const TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.bold,
                      color: Colors.greenAccent,
                    ),
                  ),
                ],
              ),
            )
            .toList(),
      ),
    );
  }

  Widget _buildMonitoringPanel() {
    final m = _monitoringData ?? MonitoringData.empty();
    final states = ["WAIT", "PREHEAT", "RUN", "INHIBIT"];
    final stateStr = m.state < states.length ? states[m.state] : "OFFLINE";
    final voltColor = m.state == 0xFF
        ? Colors.white
        : (m.voltage >= _voltageThreshold ? Colors.green : Colors.red);
    final tempColor = m.state == 0xFF
        ? Colors.white
        : (m.temperature <= _tempSetpoint ? Colors.green : Colors.red);

    return Container(
      color: Colors.grey[900],
      padding: const EdgeInsets.all(16),
      child: Column(
        children: [
          Row(
            children: [
              Expanded(child: _buildStat("STATE", stateStr, Colors.orange)),
              Expanded(
                child: _buildStat(
                  "OIL",
                  m.oilPressureOk ? "OK" : "LOW",
                  m.oilPressureOk ? Colors.green : Colors.red,
                ),
              ),
              Expanded(
                child: _buildStat(
                  "VOLT",
                  "${m.voltage.toStringAsFixed(2)}V",
                  voltColor,
                ),
              ),
              Expanded(
                child: _buildStat(
                  "TEMP",
                  "${m.temperature.toStringAsFixed(1)}°C",
                  tempColor,
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          Row(
            children: [
              Expanded(
                child: _buildStat(
                  "GRIP LEFT",
                  "${(m.dutyL / 81.91).round()}%",
                  Colors.green,
                ),
              ),
              Expanded(
                child: _buildStat(
                  "SEAT",
                  "${(m.dutyS / 81.91).round()}%",
                  Colors.green,
                ),
              ),
              Expanded(
                child: _buildStat(
                  "GRIP RIGHT",
                  "${(m.dutyR / 81.91).round()}%",
                  Colors.green,
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildStat(String label, String value, Color color) => Column(
    children: [
      Text(label, style: const TextStyle(fontSize: 10, color: Colors.grey)),
      Text(
        value,
        style: TextStyle(
          fontSize: 14,
          fontWeight: FontWeight.bold,
          color: color,
        ),
      ),
    ],
  );

  Widget _buildSection(String title) => Padding(
    padding: const EdgeInsets.symmetric(vertical: 8),
    child: Text(
      title,
      style: const TextStyle(
        fontSize: 16,
        fontWeight: FontWeight.bold,
        color: Colors.orange,
      ),
    ),
  );

  Widget _buildSlider(
    String label,
    double value,
    double min,
    double max,
    ValueChanged<double> onChanged,
  ) {
    final clamped = value.clamp(min, max);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text("$label: ${value.toStringAsFixed(2)}"),
        Slider(
          value: clamped,
          divisions: ((max - min) * 10).toInt().clamp(1, 1000),
          min: min,
          max: max,
          onChanged: onChanged,
        ),
        const Divider(),
      ],
    );
  }

  Widget _buildMiniSlider(
    String label,
    double value,
    double min,
    double max,
    ValueChanged<double> onChanged, {
    int divisions = 20,
  }) {
    final clamped = value.clamp(min, max);
    return Column(
      children: [
        Text(label, style: const TextStyle(fontSize: 10, color: Colors.grey)),
        Text(
          value.toStringAsFixed(2),
          style: const TextStyle(fontSize: 12, fontWeight: FontWeight.bold),
        ),
        Slider(
          value: clamped,
          divisions: divisions,
          min: min,
          max: max,
          onChanged: onChanged,
        ),
      ],
    );
  }

  Widget _buildIntInput(String label, int value, ValueChanged<int> onChanged) {
    return ListTile(
      title: Text(label, style: const TextStyle(fontSize: 12)),
      trailing: SizedBox(
        width: 60,
        child: TextFormField(
          initialValue: value.toString(),
          key: ValueKey("input_$label"),
          keyboardType: TextInputType.number,
          textAlign: TextAlign.end,
          style: const TextStyle(fontSize: 12),
          onChanged: (s) {
            final parsed = int.tryParse(s);
            if (parsed != null) onChanged(parsed);
          },
        ),
      ),
    );
  }
}
