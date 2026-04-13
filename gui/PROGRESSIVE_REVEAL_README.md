# FIXS Config Editor - Progressive Reveal UI Refactoring

## Overview
This refactored FIXS Config Editor implements a modern **progressive-reveal workflow** instead of showing all configuration sections at once.

## Architecture

### Components

#### 1. **ConfigCard.qml** (New)
- Reusable card component for each configuration step
- Features:
  - `cardVisible` property to control visibility (triggers fade-in + scale animation)
  - `isComplete` property to track step completion
  - `expanded` property to show/hide card contents
  - `completed()` signal emitted when user completes the step
  - `contentReady()` signal emitted when card animation completes
  - Smooth animations: opacity (300ms), scale (400ms), height transitions
  - Chevron indicator that rotates on expand/collapse

**Usage in QML:**
```qml
ConfigCard {
    id: trafficCard
    title: "Traffic Simulator"
    subtitle: "SELECT YOUR SIMULATOR"
    cardVisible: root.currentStep >= 0
    
    onCompleted: {
        root.currentStep += 1
    }
    
    // FormRow children go here...
}
```

#### 2. **FormRow.qml** (Existing)
- Two-column layout for label + control pairs
- Automatically sizes and aligns form elements

#### 3. **main.qml** (Updated)
- Main workflow orchestrator
- Tracks `currentStep` property (0-5 for 6 steps)
- Each ConfigCard has `cardVisible: root.currentStep >= <stepNumber>`
- Window starts at 400px height and grows as sections reveal
- Step progress indicator shows "Step 1 of 6", etc.
- ScrollView allows viewing multiple revealed sections

#### 4. **main.py** (Existing, no changes)
- ConfigManager provides backend:
  - `loadConfig()` - open YAML file
  - `saveConfig()` - save to YAML
  - `setField(section, field, value)` - update config data
  - `statusUpdated` signal - status messages

## Workflow

### Initial Load
```
User sees: Traffic Simulator card only (Step 1 of 6)
Window height: ~400px
Card opacity: 1.0, scale: 1.0 (fully visible)
```

### Step Completion Pattern
```
1. User selects traffic simulator from dropdown
2. simulatorCombo.onCurrentTextChanged fired
3. Calls trafficCard.markComplete()
4. trafficCard.completed() signal emitted
5. root.currentStep incremented (0 → 1)
6. appSetupCard.cardVisible becomes true
7. appSetupCard animates in: opacity 0→1, scale 0.95→1.0
8. Window might resize to show more content
9. Step indicator updates to "Step 2 of 6"
```

### Animation Details
- **Fade-in:** Opacity 0 → 1 (400ms, OutCubic easing)
- **Scale-in:** Scale 0.95 → 1.0 (400ms, OutCubic easing)
- **Height:** Auto-animates as content expands (300ms, OutCubic easing)
- **Chevron rotation:** -180° → 0° when expanding (200ms, OutCubic easing)

## Configuration Steps

1. **Traffic Simulator** (Step 0)
   - Trigger: Select simulator from ComboBox
   - Fields: Enable Real-Sim, Simulator Type, IP, Port

2. **Application Setup** (Step 1)
   - Trigger: Enter Host IP (length > 0)
   - Fields: Host IP, Host Port

3. **Application Layer** (Step 2)
   - Trigger: Enter Application Name (length > 0)
   - Fields: App Name, Enable App, App Port

4. **XIL Bridge** (Step 3)
   - Trigger: Select XIL Simulator from ComboBox
   - Fields: Enable XIL, XIL Simulator, XIL IP, XIL Port

5. **CARLA** (Step 4)
   - Trigger: Enter CARLA IP (length > 0)
   - Fields: Enable CARLA, CARLA IP, CARLA Port

6. **IPG CarMaker** (Step 5)
   - Trigger: Select CarMaker Version from ComboBox
   - Fields: Enable CarMaker, CarMaker Version, IP, Port

## Key Features

✅ **Progressive Reveal** - One card at a time, guided workflow  
✅ **Smooth Animations** - Fade + scale for professional feel  
✅ **No Janky Layout** - Smooth height transitions, no jumping  
✅ **Reusable Components** - ConfigCard.qml can be used anywhere  
✅ **State Machine** - currentStep property drives visibility  
✅ **Clean Architecture** - QML handles UI state, Python handles data  
✅ **Extensible** - Easy to add/remove/modify steps  

## How to Extend

### Add a New Step

1. Add property to root for tracking (optional):
   ```qml
   property bool newStepComplete: false
   ```

2. Create new ConfigCard:
   ```qml
   ConfigCard {
       id: newCard
       title: "New Step"
       subtitle: "DESCRIPTION"
       cardVisible: root.currentStep >= 6
       
       onCompleted: {
           root.currentStep += 1
           root.newStepComplete = true
       }
       
       // Fields...
   }
   ```

3. Update `maxSteps: 7` (was 6)

### Modify Completion Trigger

Each card calls `markComplete()` when a condition is met:

```qml
// Example: Auto-complete on checkbox check
CheckBox {
    onCheckedChanged: {
        if (checked && root.currentStep === 2) {
            appLayerCard.markComplete()
        }
    }
}
```

### Wire in Real Validation

Currently using simple checks (text length > 0, index >= 0). Replace with real logic:

```qml
TextField {
    onEditingFinished: {
        if (isValidIP(text) && root.currentStep === 4) {
            carlaCard.markComplete()
        }
    }
}
```

## Visual Behavior

- **Width:** 800px (fixed)
- **Height:** 400px minimum, grows with revealed cards
- **Cards:** ~90px collapsed header, ~250-350px when expanded
- **Colors:** Light blue background (#F0F4FF), soft borders (#E0E8FF)
- **Spacing:** 16px between cards, 12px within card form rows
- **No scrollbar emphasis** - Scrolling is smooth and natural as user adds input

## Notes

- Cards remain on the same page (no dialogs or tabs)
- Window can be resized by user
- Load/Save buttons stay at top
- Status label shows at bottom
- Step indicator helps user track progress
- All state is in root properties (easy to debug/inspect)
