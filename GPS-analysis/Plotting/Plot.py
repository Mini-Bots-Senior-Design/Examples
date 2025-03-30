import matplotlib.pyplot as plt

# Sample GPS coordinates (latitude, longitude)
gps_coordinates = [
    (42.3493889, -71.1059071),  # Start point
    (42.3514134, -71.1007750),  # Another point
    (42.3525000, -71.0950000),  # Another point
    (42.3550000, -71.0900000),  # Destination
]

# Extract latitude and longitude separately
latitudes = [coord[0] for coord in gps_coordinates]
longitudes = [coord[1] for coord in gps_coordinates]

# Create the plot
plt.figure(figsize=(8, 6))
plt.scatter(longitudes, latitudes, c='red', label="GPS Points")
plt.plot(longitudes, latitudes, linestyle='dotted', color='blue', label="Path")

# Add labels and grid
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("GPS Path Plot")
plt.legend()
plt.grid(True)

# Show the plot
plt.show()