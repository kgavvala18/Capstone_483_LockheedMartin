// renderer.js

// Access Mousetrap via the exposed global.
const Mousetrap = window.Mousetrap

// Object to hold keybind mappings for five sensors
let keybindMappings = {
  flex_thumb: '',
  flex_index: '',
  flex_middle: '',
  flex_ring: '',
  flex_pinky: ''
}

// Update mappings from input fields and refresh Mousetrap bindings
function updateKeybindMappings() {
  keybindMappings.flex_thumb = document.getElementById('flex_thumb').value.trim()
  keybindMappings.flex_index = document.getElementById('flex_index').value.trim()
  keybindMappings.flex_middle = document.getElementById('flex_middle').value.trim()
  keybindMappings.flex_ring = document.getElementById('flex_ring').value.trim()
  keybindMappings.flex_pinky = document.getElementById('flex_pinky').value.trim()
  console.log("Saved keybinds:", keybindMappings)
  
  updateMousetrapBindings()
  // Optionally, persist these settings using localStorage or IPC
}

// Update Mousetrap bindings based on keybindMappings
function updateMousetrapBindings() {
  // First, unbind all existing keybinds from Mousetrap
  for (let sensor in keybindMappings) {
    if (keybindMappings[sensor]) {
      Mousetrap.unbind(keybindMappings[sensor])
    }
  }
  
  // Bind new key sequences for each sensor
  for (let sensor in keybindMappings) {
    const keybind = keybindMappings[sensor]
    if (keybind) {
      Mousetrap.bind(keybind, function(e) {
        triggerSensor(sensor)
        e.preventDefault()
      })
    }
  }
}

// Save button event listener
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
