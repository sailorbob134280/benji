<script>
  import Slide from '../lib/Slide.svelte';
</script>

<section data-background="assets/BG2.png" data-background-opacity="0.3">
  <h1 class="font-sans font-thin text-8xl">Architecture</h1>
</section>

<Slide title="Physical Design">
  <div class="grid grid-cols-2 gap-8 text-xl">
    <div>
      <img src="assets/benji.jpg" alt="Benji robot" class="rounded-lg shadow-lg max-h-80 mx-auto" />
      <div>
        <h3 class="text-2xl font-semibold mb-2 text-purple-400">Integration</h3>
        <ul class="list-disc list-inside space-y-1">
          <li>Custom PCB for MCU, motor drivers, and sensors</li>
          <li>RasPi Expansion Header compatible</li>
        </ul>
      </div>
    </div>
    <div class="space-y-4">
      <div>
        <h3 class="text-2xl font-semibold mb-2 text-yellow-400">Actuators</h3>
        <ul class="list-disc list-inside space-y-1">
          <li>4x DC gearmotors, Differential Drive</li>
          <li>Servo for camera tilt</li>
        </ul>
      </div>
      <div>
        <h3 class="text-2xl font-semibold mb-2 text-blue-400">Sensors</h3>
        <ul class="list-disc list-inside space-y-1">
          <li>6-axis IMU (LS6DSMTR) + 3-axis magnetometer (LIS2MDLTR)</li>
          <li>4x quadrature wheel encoders</li>
          <li>Linear ToF distance sensor (VL35L1X)</li>
          <li>Camera (RasPi CM3)</li>
        </ul>
      </div>
      <div>
        <h3 class="text-2xl font-semibold mb-2 text-green-400">Compute</h3>
        <ul class="list-disc list-inside space-y-1">
          <li><span class="font-mono">STM32F405</span> - Real-time control</li>
          <li><span class="font-mono">Raspberry Pi 5</span> - Vision, planning, and API</li>
        </ul>
      </div>
  </div>
</Slide>

<Slide title="Software Stack">
  <div class="grid grid-cols-2 gap-6">
    <div>
      <img src="assets/benji_sw_arch.png" alt="Benji software architecture diagram" />
    </div>
    <div class="text-lg space-y-4">
      <div>
        <h3 class="text-xl font-semibold mb-2 text-blue-400">Compute - Yocto Linux</h3>
        <ul class="list-disc list-inside space-y-1">
          <li>Custom distro, via Kas (AutonomOS via Kas)</li>
          <li>Docker Support</li>
          <li>OTA Updates via Mender</li>
          <li>C++/Go Microservices over NATS</li>
        </ul>
        <p class="text-base text-gray-400">SLAM, path planning, API/UI</p>
      </div>
      <div>
        <h3 class="text-xl font-semibold mb-2 text-yellow-400">RT Core - Zephyr RTOS</h3>
        <ul class="list-disc list-inside space-y-1">
          <li>App-layer components in C++</li>
          <li>Command/Telem via Protobuf or debug shell</li>
          <li>Debug shell, DFU updates</li>
        </ul>
        <p class="text-base text-gray-400">Sensor acquisition, actuator control, short-horizon GNC</p>
      </div>
      <div class="text-base">
        <span class="text-gray-400">Link:</span>
        <span class="font-mono text-green-400">Protobuf</span> over UART with custom framing
      </div>
    </div>
  </div>
</Slide>

<Slide title="RT-Compute Protocol">
  <div class="grid grid-cols-2 gap-8">
    <div class="space-y-6">
      <div>
        <h3 class="text-2xl font-semibold mb-3 text-purple-400">Interaction Model</h3>
        <ul class="list-disc list-inside space-y-2 text-lg">
          <li><span class="text-yellow-300">Controller-Peripheral</span> pattern</li>
          <li>Compute is controller, RT core is peripheral</li>
          <li>RT core is expected to handle one interaction at a time</li>
          <li>Compute will periodically fetch telemetry of interest</li>
        </ul>
      </div>
      <div>
        <h3 class="text-2xl font-semibold mb-3 text-blue-400">Interaction Types</h3>
        <ul class="list-disc list-inside space-y-2 text-lg">
          <li><span class="text-green-400 font-semibold">Command</span> - Action request, returns status</li>
          <li><span class="text-green-400 font-semibold">Request/Response</span> - Data query, returns payload or error</li>
        </ul>
        <p class="text-base text-gray-400 mt-2 ml-6">No streaming - completion signaled by response</p>
      </div>
    </div>
    <div class="space-y-6">
      <div>
        <h3 class="text-2xl font-semibold mb-3 text-orange-400">Frame Format</h3>
        <div class="bg-gray-800/50 rounded-lg p-4 font-mono text-base">
          <div class="grid grid-cols-4 gap-2 text-center mb-2">
            <div class="bg-purple-600/40 rounded p-2">Sync</div>
            <div class="bg-blue-600/40 rounded p-2">Length</div>
            <div class="bg-green-600/40 rounded p-2">Data</div>
            <div class="bg-yellow-600/40 rounded p-2">CRC16</div>
          </div>
          <div class="grid grid-cols-4 gap-2 text-center text-sm text-gray-400">
            <div>4 bytes</div>
            <div>4 bytes</div>
            <div>variable</div>
            <div>2 bytes</div>
          </div>
        </div>
        <ul class="list-disc list-inside space-y-1 text-base mt-3">
          <li><span class="text-purple-400">Sync:</span> <span class="font-mono">0x352ef853</span> (CCSDS embedded sync marker)</li>
          <li><span class="text-blue-400">Length:</span> Little-endian, data field only</li>
          <li><span class="text-green-400">Data:</span> Protobuf-encoded message</li>
          <li><span class="text-yellow-400">CRC16:</span> Checksum of data field</li>
        </ul>
      </div>
    </div>
  </div>
</Slide>

