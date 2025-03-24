// renderer.js

// Object to hold keybind mappings for five sensors
let keybindMappings = {
    flex_thumb: '',
    flex_index: '',
    flex_middle: '',
    flex_ring: '',
    flex_pinky: ''
  }
  
  // Update mappings from input fields
  function updateKeybindMappings() {
    keybindMappings.flex_thumb = document.getElementById('flex_thumb').value.trim()
    keybindMappings.flex_index = document.getElementById('flex_index').value.trim()
    keybindMappings.flex_middle = document.getElementById('flex_middle').value.trim()
    keybindMappings.flex_ring = document.getElementById('flex_ring').value.trim()
    keybindMappings.flex_pinky = document.getElementById('flex_pinky').value.trim()
    console.log("Saved keybinds:", keybindMappings)
    // Persist settings if needed (e.g., localStorage or IPC call)
  }
  
  document.getElementById('saveBindings').addEventListener('click', updateKeybindMappings)
  
  // Simulated sensor trigger function
  function triggerSensor(sensorName) {
    console.log(`${sensorName} triggered`)
    document.getElementById('handImage').src = `images/hand_${sensorName}.png`
    setTimeout(() => {
      document.getElementById('handImage').src = 'images/hand_default.png'
    }, 1000)
  }
  
  // Add event listeners for simulation buttons
  document.querySelectorAll('.sensor-btn').forEach(button => {
    button.addEventListener('click', (e) => {
      const sensor = e.target.getAttribute('data-sensor')
      triggerSensor(sensor)
    })
  })
  
  // Example: Listening for keydown events to simulate sensor activation via keybinds
  document.addEventListener('keydown', (e) => {
    let keys = []
    if (e.ctrlKey) keys.push('Ctrl')
    if (e.altKey) keys.push('Alt')
    if (e.shiftKey) keys.push('Shift')
    keys.push(e.key)
    const pressedCombo = keys.join('+')
  
    // Check each sensor mapping
    for (const sensor in keybindMappings) {
      if (keybindMappings[sensor] && pressedCombo.toLowerCase() === keybindMappings[sensor].toLowerCase()) {
        triggerSensor(sensor)
        // Prevent default behavior for system key sequences (e.g., Alt+F4)
        e.preventDefault()
        break
      }
    }
  })
  