import os
import urllib.request
import math
import time

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def download_file(url, filepath):
    if os.path.exists(filepath):
        print(f"Already exists: {filepath}")
        return
    print(f"Downloading: {url} -> {filepath}")
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'})
    try:
        with urllib.request.urlopen(req) as response, open(filepath, 'wb') as out_file:
            out_file.write(response.read())
        time.sleep(0.1) # Be polite to OSM servers
    except Exception as e:
        print(f"Failed to download {url}: {e}")

# Base directory for offline assets
BASE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'assets')

def download_leaflet():
    leaflet_dir = os.path.join(BASE_DIR, 'leaflet')
    # Leaflet core
    download_file('https://unpkg.com/leaflet@1.9.4/dist/leaflet.css', os.path.join(leaflet_dir, 'leaflet.css'))
    download_file('https://unpkg.com/leaflet@1.9.4/dist/leaflet.js', os.path.join(leaflet_dir, 'leaflet.js'))
    
    # Leaflet images (markers etc)
    images_dir = os.path.join(leaflet_dir, 'images')
    images = [
        'marker-icon.png', 'marker-icon-2x.png', 'marker-shadow.png',
        'layers.png', 'layers-2x.png'
    ]
    for img in images:
        download_file(f'https://unpkg.com/leaflet@1.9.4/dist/images/{img}', os.path.join(images_dir, img))

def download_tiles():
    # Konkuk University Bounding Box
    lat_min, lat_max = 37.535, 37.548
    lon_min, lon_max = 127.068, 127.085
    zoom_levels = [16, 17, 18, 19]

    tiles_dir = os.path.join(BASE_DIR, 'tiles')
    
    total_tiles = 0
    for zoom in zoom_levels:
        x_min, y_max = deg2num(lat_min, lon_min, zoom)
        x_max, y_min = deg2num(lat_max, lon_max, zoom)
        
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                total_tiles += 1
                
    print(f"Starting download of {total_tiles} map tiles...")
    
    count = 0
    for zoom in zoom_levels:
        x_min, y_max = deg2num(lat_min, lon_min, zoom)
        x_max, y_min = deg2num(lat_max, lon_max, zoom)
        
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                url = f"https://tile.openstreetmap.org/{zoom}/{x}/{y}.png"
                filepath = os.path.join(tiles_dir, str(zoom), str(x), f"{y}.png")
                download_file(url, filepath)
                count += 1
                if count % 50 == 0:
                    print(f"Progress: {count}/{total_tiles}")

if __name__ == "__main__":
    print("Downloading Leaflet assets...")
    download_leaflet()
    print("Downloading OSM Map Tiles for Konkuk University...")
    download_tiles()
    print("Download completed!")
