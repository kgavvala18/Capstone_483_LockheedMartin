// renderer.js

// Object to hold keybind mappings
let keybindMappings = {
    flex_thumb: '',
    flex_index: '',
    flex_middle: ''
    // add additional sensors as needed
  }
  
  // Utility: Update stored mapping from input fields
  function updateKeybindMappings() {
    keybindMappings.flex_thumb = document.getElementById('flex_thumb').value.trim()
    keybindMappings.flex_index = document.getElementById('flex_index').value.trim()
    keybindMappings.flex_middle = document.getElementById('flex_middle').value.trim()
    console.log("Saved keybinds:", keybindMappings)
    // You could persist these settings using localStorage or IPC to save configuration.
  }
  
  // Save button event listener
  document.getElementById('saveBindings').addEventListener('click', updateKeybindMappings)
  
  // Simulated sensor trigger handler
  function triggerSensor(sensorName) {
    console.log(`${sensorName} triggered`)
    // Update the hand image based on the sensor triggered
    // We assume you have an image for each sensor like hand_flex_thumb.png
    document.getElementById('handImage').src = `images/hand_${sensorName}.png`
    // Optionally, reset back to default after a short delay
    setTimeout(() => {
      document.getElementById('handImage').src = 'images/hand_default.png'
    }, 1000)
  }
  
  // Add event listeners to simulate sensor buttons
  document.querySelectorAll('.sensor-btn').forEach(button => {
    button.addEventListener('click', (e) => {
      const sensor = e.target.getAttribute('data-sensor')
      triggerSensor(sensor)
    })
  })
  
  // Example: Listening for keydown events to simulate sensor activation via keybinds
  document.addEventListener('keydown', (e) => {
    // Build a string representing the key sequence pressed
    // (This example simply combines modifiers and the key; for more complex sequences, use a library)
    let keys = []
    if (e.ctrlKey) keys.push('Ctrl')
    if (e.altKey) keys.push('Alt')
    if (e.shiftKey) keys.push('Shift')
    // Using the event.key value directly; you might want to standardize this (e.g., converting to uppercase)
    keys.push(e.key)
    const pressedCombo = keys.join('+')
    
    // Check if the pressed combination matches any sensor mapping
    for (const sensor in keybindMappings) {
      if (keybindMappings[sensor] && pressedCombo.toLowerCase() === keybindMappings[sensor].toLowerCase()) {
        triggerSensor(sensor)
        // Optionally, prevent default behavior if it’s a system key sequence (like Alt+F4)
        e.preventDefault()
        break
      }
    }
  })
  