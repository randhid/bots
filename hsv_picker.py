import cv2
import numpy as np

def nothing(x):
    pass

# Create a window
cv2.namedWindow('HSV Picker')

# Create trackbars for HSV values
cv2.createTrackbar('H_min', 'HSV Picker', 0, 179, nothing)
cv2.createTrackbar('S_min', 'HSV Picker', 0, 255, nothing)
cv2.createTrackbar('V_min', 'HSV Picker', 0, 255, nothing)
cv2.createTrackbar('H_max', 'HSV Picker', 179, 179, nothing)
cv2.createTrackbar('S_max', 'HSV Picker', 255, 255, nothing)
cv2.createTrackbar('V_max', 'HSV Picker', 255, 255, nothing)

# Set default values for green (approximate)
cv2.setTrackbarPos('H_min', 'HSV Picker', 35)  # Green hue starts around 35
cv2.setTrackbarPos('H_max', 'HSV Picker', 85)  # Green hue ends around 85
cv2.setTrackbarPos('S_min', 'HSV Picker', 50)  # Minimum saturation
cv2.setTrackbarPos('V_min', 'HSV Picker', 50)  # Minimum value

# Initialize camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera")
    exit()

print("HSV Picker for Green Objects")
print("Adjust the sliders to match your green object")
print("Press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame")
        break
    
    # Get current trackbar positions
    h_min = cv2.getTrackbarPos('H_min', 'HSV Picker')
    s_min = cv2.getTrackbarPos('S_min', 'HSV Picker')
    v_min = cv2.getTrackbarPos('V_min', 'HSV Picker')
    h_max = cv2.getTrackbarPos('H_max', 'HSV Picker')
    s_max = cv2.getTrackbarPos('S_max', 'HSV Picker')
    v_max = cv2.getTrackbarPos('V_max', 'HSV Picker')
    
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create mask
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)
    
    # Apply mask to original frame
    result = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Display HSV values on frame
    cv2.putText(frame, f'H: {h_min}-{h_max}, S: {s_min}-{s_max}, V: {v_min}-{v_max}', 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Show frames
    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)
    
    # Break on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

print(f"\nFinal HSV values for green:")
print(f"Lower: [{h_min}, {s_min}, {v_min}]")
print(f"Upper: [{h_max}, {s_max}, {v_max}]")
print(f"\nFor your code, use:")
print(f"lower = np.array([{h_min}, {s_min}, {v_min}])")
print(f"upper = np.array([{h_max}, {s_max}, {v_max}])") 
