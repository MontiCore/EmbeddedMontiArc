<template>
  <div class="d-flex flex-column w-75 m-auto gap-3">
    <h1 class="text-start m-0">{{ title }}</h1>

    <p class="text-start m-0">{{ description }}</p>

    <div id="map" style="height: 600px"></div>

    <div class="container">
      <div class="row">
        <div class="col-5 d-flex align-items-center ps-0">
          <Icon icon="bxs:truck" style="font-size: 1.5rem" />
          <p class="m-0 text-nowrap">Trucks</p>
        </div>
        <div class="col-1 text-end">123</div>
        <div class="col-5 d-flex align-items-center">
          <Icon icon="jam:triangle-danger" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">Bad driver score</p>
        </div>
        <div class="col-1 text-end pe-0">17</div>
      </div>
      <div class="row">
        <div class="col-5 d-flex align-items-center ps-0">
          <Icon icon="bxs:gas-pump" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">Avg. fuel consumption</p>
        </div>
        <div class="col-1 text-end">35</div>
        <div class="col-5 d-flex align-items-center">
          <Icon icon="mdi:tools" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">Repair needed</p>
        </div>
        <div class="col-1 text-end pe-0">4</div>
      </div>
      <div class="row">
        <div class="col-5 d-flex align-items-center ps-0">
          <Icon icon="mdi:highway" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">Active</p>
        </div>
        <div class="col-1 text-end">100</div>
        <div class="col-5 d-flex align-items-center">
          <Icon icon="mdi-light:sleep" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">Resting</p>
        </div>
        <div class="col-1 text-end pe-0">117</div>
      </div>
    </div>

    <div style="overflow-x: scroll">
      <table class="table table-hover">
        <thead>
          <tr>
            <th class="text-nowrap" v-for="(_, key) in dataset[0]" :key="key">
              {{ key }}
            </th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="row in dataset" :key="row">
            <td
              class="text-nowrap"
              v-for="(value, key) in row"
              :key="key"
              style="border-width: 0"
            >
              {{ value }}
            </td>
          </tr>
        </tbody>
      </table>

    </div>
  </div>
</template>

<script>
import { Icon } from '@iconify/vue'
import leaflet from 'leaflet'
import HeatmapOverlay from 'leaflet-heatmap'

export default {
  components: {
    Icon
  },
  data () {
    return {
      title: 'Dachser Western Germany',
      description:
        'Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet. Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet.',
      dataset: [
        {
          Id: 149731414,
          Longitude: 9.4835,
          Latitude: 47.6559,
          GpsTime: '2021-04-30T14:01:38Z',
          Heading: 2,
          Speed: 0,
          OdoMeter: 5447,
          TotalFuelUsedDemiLiters: 1,
          ValidPosition: true,
          Timestamp: '2021-04-30T14:01:27.7866404Z',
          DIdCIdDay: '95e1053d34c316cbde5c253ac750a6f5'
        },
        {
          Id: 5463436,
          Longitude: 19.4835,
          Latitude: 44.6559,
          GpsTime: '2022-04-30T14:01:38Z',
          Heading: 2,
          Speed: 0,
          OdoMeter: 5447,
          TotalFuelUsedDemiLiters: 1,
          ValidPosition: true,
          Timestamp: '2021-04-30T14:01:27.7866404Z',
          DIdCIdDay: '95e1053d34c316cbde5c253ac750a6f5'
        }
      ]
    }
  },
  mounted () {
    const CENTER_COORDINATES_GERMANY = [51.1657, 10.4515]
    const map = leaflet.map('map').setView(CENTER_COORDINATES_GERMANY, 6)
    leaflet.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=pk.eyJ1IjoianVibGExMDIiLCJhIjoiY2wxZXMzZ3Z6MGx6YzNjbG45bGd1ZjV3ciJ9.QDU9GpeOQSK-zSCTb-lLTg', {
      attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
      minZoom: 2,
      maxZoom: 18,
      id: 'mapbox/streets-v11',
      tileSize: 512,
      zoomOffset: -1,
      accessToken: 'pk.eyJ1IjoianVibGExMDIiLCJhIjoiY2wxZXMzZ3Z6MGx6YzNjbG45bGd1ZjV3ciJ9.QDU9GpeOQSK-zSCTb-lLTg'
    }).addTo(map)
    leaflet.marker(CENTER_COORDINATES_GERMANY).addTo(map)

    const cfg = {
      // radius should be small ONLY if scaleRadius is true (or small radius is intended)
      // if scaleRadius is false it will be the constant radius used in pixels
      radius: 0.3,
      maxOpacity: 0.7,
      // scales the radius based on map zoom
      scaleRadius: true,
      // if set to false the heatmap uses the global maximum for colorization
      // if activated: uses the data maximum within the current map boundaries
      //   (there will always be a red spot with useLocalExtremas true)
      // useLocalExtrema: false,
      // which field name in your data represents the latitude - default "lat"
      latField: 'lat',
      // which field name in your data represents the longitude - default "lng"
      lngField: 'lng',
      // which field name in your data represents the data value - default "value"
      valueField: 'count'
    }
    const testData = {
      data: [
        { lat: 48.51, lng: 10.06 },
        { lat: 50.45, lng: 6.06 },
        { lat: 52.56, lng: 9.12 },
        { lat: 49.26, lng: 11.52 },
        { lat: 48.29, lng: 11.55 },
        { lat: 49.28, lng: 10.34 },
        { lat: 51.24, lng: 8.05 },
        { lat: 49.58, lng: 9.06 },
        { lat: 51.45, lng: 11.29 },
        { lat: 48.25, lng: 20.20 },
        { lat: 53.28, lng: 7.28 },
        { lat: 50.11, lng: 10.04 },
        { lat: 48.44, lng: 8.13 },
        { lat: 48.20, lng: 8.40 },
        { lat: 49.54, lng: 10.54 },
        { lat: 51.10, lng: 14.26 },
        { lat: 48.50, lng: 12.00 },
        { lat: 48.50, lng: 12.00 },
        { lat: 49.56, lng: 11.35 },
        { lat: 50.59, lng: 7.08 },
        { lat: 52.30, lng: 13.25 },
        { lat: 51.47, lng: 11.44 },
        { lat: 48.05, lng: 9.47 },
        { lat: 52.01, lng: 8.33 },
        { lat: 48.30, lng: 8.20 },
        { lat: 51.28, lng: 7.13 },
        { lat: 49.08, lng: 13.14 },
        { lat: 49.08, lng: 13.14 },
        { lat: 50.46, lng: 7.06 },
        { lat: 53.34, lng: 6.40 },
        { lat: 51.31, lng: 6.58 },
        { lat: 53.33, lng: 13.15 },
        { lat: 52.25, lng: 12.33 },
        { lat: 52.50, lng: 13.00 },
        { lat: 52.15, lng: 10.31 },
        { lat: 53.04, lng: 8.47 },
        { lat: 53.33, lng: 8.36 },
        { lat: 51.47, lng: 10.37 },
        { lat: 52.15, lng: 10.31 },
        { lat: 53.28, lng: 9.39 },
        { lat: 52.37, lng: 10.04 },
        { lat: 50.51, lng: 12.54 },
        { lat: 47.53, lng: 12.28 },
        { lat: 50.15, lng: 10.58 },
        { lat: 50.56, lng: 6.57 },
        { lat: 47.40, lng: 9.10 },
        { lat: 51.45, lng: 14.20 },
        { lat: 49.08, lng: 10.05 },
        { lat: 53.51, lng: 8.41 },
        { lat: 48.15, lng: 11.26 },
        { lat: 49.51, lng: 8.39 },
        { lat: 48.50, lng: 12.57 },
        { lat: 51.51, lng: 12.14 },
        { lat: 51.56, lng: 8.52 },
        { lat: 54.15, lng: 8.00 },
        { lat: 48.43, lng: 10.47 },
        { lat: 51.30, lng: 7.28 },
        { lat: 51.03, lng: 13.44 },
        { lat: 51.26, lng: 6.45 },
        { lat: 50.48, lng: 6.29 },
        { lat: 51.14, lng: 6.47 },
        { lat: 52.50, lng: 13.49 },
        { lat: 51.12, lng: 9.28 },
        { lat: 50.15, lng: 6.50 },
        { lat: 50.58, lng: 10.19 },
        { lat: 53.07, lng: 11.15 },
        { lat: 53.43, lng: 9.40 },
        { lat: 53.21, lng: 7.12 },
        { lat: 53.20, lng: 7.12 },
        { lat: 50.58, lng: 11.02 },
        { lat: 49.36, lng: 11.00 },
        { lat: 50.27, lng: 12.55 },
        { lat: 51.28, lng: 7.02 },
        { lat: 48.44, lng: 9.18 },
        { lat: 54.27, lng: 11.07 },
        { lat: 54.47, lng: 9.27 },
        { lat: 54.43, lng: 8.30 },
        { lat: 51.45, lng: 14.37 },
        { lat: 52.20, lng: 14.32 },
        { lat: 50.07, lng: 8.41 },
        { lat: 49.10, lng: 11.23 },
        { lat: 47.59, lng: 7.51 },
        { lat: 48.24, lng: 11.45 },
        { lat: 47.39, lng: 9.30 },
        { lat: 50.32, lng: 9.40 },
        { lat: 51.25, lng: 9.39 },
        { lat: 52.22, lng: 14.03 },
        { lat: 49.28, lng: 10.59 },
        { lat: 47.30, lng: 11.06 },
        { lat: 53.26, lng: 10.22 },
        { lat: 51.32, lng: 7.06 },
        { lat: 50.53, lng: 12.04 },
        { lat: 50.34, lng: 8.41 },
        { lat: 48.42, lng: 9.39 },
        { lat: 51.09, lng: 14.58 },
        { lat: 51.54, lng: 10.25 },
        { lat: 50.56, lng: 10.42 },
        { lat: 51.31, lng: 9.55 },
        { lat: 54.05, lng: 13.23 },
        { lat: 50.59, lng: 12.10 },
        { lat: 49.06, lng: 13.08 },
        { lat: 53.47, lng: 12.10 },
        { lat: 51.54, lng: 8.24 },
        { lat: 51.21, lng: 7.27 },
        { lat: 51.54, lng: 11.03 },
        { lat: 51.30, lng: 11.56 },
        { lat: 53.33, lng: 9.59 },
        { lat: 52.06, lng: 9.21 },
        { lat: 52.06, lng: 9.21 },
        { lat: 51.40, lng: 7.50 },
        { lat: 50.07, lng: 8.56 },
        { lat: 52.22, lng: 9.46 },
        { lat: 52.22, lng: 9.46 },
        { lat: 51.38, lng: 10.44 },
        { lat: 52.50, lng: 12.03 },
        { lat: 49.24, lng: 8.42 },
        { lat: 49.09, lng: 9.13 },
        { lat: 54.10, lng: 7.53 },
        { lat: 54.10, lng: 7.53 },
        { lat: 54.15, lng: 8.00 },
        { lat: 52.07, lng: 8.39 },
        { lat: 51.32, lng: 7.14 },
        { lat: 50.30, lng: 9.00 },
        { lat: 50.30, lng: 9.00 },
        { lat: 52.09, lng: 9.56 },
        { lat: 50.19, lng: 11.55 },
        { lat: 50.24, lng: 9.58 },
        { lat: 51.46, lng: 9.22 },
        { lat: 51.26, lng: 14.14 },
        { lat: 49.56, lng: 7.27 },
        { lat: 49.43, lng: 7.16 },
        { lat: 48.23, lng: 9.58 },
        { lat: 48.46, lng: 11.26 },
        { lat: 48.48, lng: 12.57 },
        { lat: 53.55, lng: 9.31 },
        { lat: 50.54, lng: 11.35 },
        { lat: 48.20, lng: 9.30 },
        { lat: 49.26, lng: 7.45 },
        { lat: 49.00, lng: 8.23 },
        { lat: 51.18, lng: 9.26 },
        { lat: 47.45, lng: 10.17 },
        { lat: 54.19, lng: 10.08 },
        { lat: 54.12, lng: 9.32 },
        { lat: 54.35, lng: 10.25 },
        { lat: 50.21, lng: 7.36 },
        { lat: 50.56, lng: 6.57 },
        { lat: 47.40, lng: 9.10 },
        { lat: 51.20, lng: 6.33 },
        { lat: 50.19, lng: 7.37 },
        { lat: 48.34, lng: 12.08 },
        { lat: 51.29, lng: 13.47 },
        { lat: 48.43, lng: 10.56 },
        { lat: 53.13, lng: 7.26 },
        { lat: 52.43, lng: 9.36 },
        { lat: 51.18, lng: 12.22 },
        { lat: 50.22, lng: 8.04 },
        { lat: 52.31, lng: 7.19 },
        { lat: 51.39, lng: 6.36 },
        { lat: 53.52, lng: 10.40 },
        { lat: 52.05, lng: 13.10 },
        { lat: 48.53, lng: 9.11 },
        { lat: 49.29, lng: 8.26 },
        { lat: 53.15, lng: 10.24 },
        { lat: 53.10, lng: 10.12 },
        { lat: 53.10, lng: 10.12 },
        { lat: 51.53, lng: 12.39 },
        { lat: 52.07, lng: 11.38 },
        { lat: 50.01, lng: 8.14 },
        { lat: 49.29, lng: 8.29 },
        { lat: 53.33, lng: 11.40 },
        { lat: 54.20, lng: 11.40 },
        { lat: 51.09, lng: 13.29 },
        { lat: 47.58, lng: 10.10 },
        { lat: 51.11, lng: 6.27 },
        { lat: 51.12, lng: 10.27 },
        { lat: 51.53, lng: 12.15 },
        { lat: 51.52, lng: 6.54 },
        { lat: 48.08, lng: 11.34 },
        { lat: 51.11, lng: 6.27 },
        { lat: 51.25, lng: 9.38 },
        { lat: 48.08, lng: 11.31 },
        { lat: 51.58, lng: 7.37 },
        { lat: 51.09, lng: 11.47 },
        { lat: 49.27, lng: 8.29 },
        { lat: 54.04, lng: 9.58 },
        { lat: 49.20, lng: 7.09 },
        { lat: 52.55, lng: 12.48 },
        { lat: 52.39, lng: 9.13 },
        { lat: 54.12, lng: 9.32 },
        { lat: 53.42, lng: 7.09 },
        { lat: 53.42, lng: 10.01 },
        { lat: 54.40, lng: 8.20 },
        { lat: 51.30, lng: 10.47 },
        { lat: 51.45, lng: 7.30 },
        { lat: 54.40, lng: 8.20 },
        { lat: 51.45, lng: 7.30 },
        { lat: 49.27, lng: 11.03 },
        { lat: 49.27, lng: 11.03 },
        { lat: 51.28, lng: 6.51 },
        { lat: 50.06, lng: 8.44 },
        { lat: 48.28, lng: 7.56 },
        { lat: 53.09, lng: 8.13 },
        { lat: 52.45, lng: 13.14 },
        { lat: 50.27, lng: 12.55 },
        { lat: 52.17, lng: 8.03 },
        { lat: 51.42, lng: 8.45 },
        { lat: 53.26, lng: 11.52 },
        { lat: 48.34, lng: 13.28 },
        { lat: 52.19, lng: 10.14 },
        { lat: 48.52, lng: 8.41 },
        { lat: 49.12, lng: 7.36 },
        { lat: 50.30, lng: 12.08 },
        { lat: 48.46, lng: 13.51 },
        { lat: 52.25, lng: 13.04 },
        { lat: 54.30, lng: 11.10 },
        { lat: 52.37, lng: 12.19 },
        { lat: 47.46, lng: 9.36 },
        { lat: 51.7, lng: 7.12 },
        { lat: 49.01, lng: 12.06 },
        { lat: 50.37, lng: 12.17 },
        { lat: 51.11, lng: 7.12 },
        { lat: 54.17, lng: 9.39 },
        { lat: 48.29, lng: 9.12 },
        { lat: 52.17, lng: 7.26 }
      ]
    }

    const heatmapLayer = new HeatmapOverlay(cfg)
    heatmapLayer.setData(testData)
    heatmapLayer.addTo(map)
  }
}
</script>

<style scoped>
.table > :not(:first-child) {
  border-top: 0;
}
</style>
