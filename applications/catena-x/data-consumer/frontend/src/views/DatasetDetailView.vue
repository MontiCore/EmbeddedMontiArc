<template>
  <div class="d-flex flex-column align-items-center gap-3 m-4">
    <div v-if="alert" class="alert alert-danger mt-4" role="alert">
      <strong>Policy violation!</strong> Data access not possible!<br>
      <a href="/datasets" class="alert-link">Return to datasets overview</a>
    </div>
    <div v-else class="d-flex flex-column m-auto gap-3 w-100">
      <div class="d-flex justify-content-between">
      <div class="d-flex flex-column gap-3 w-75">
      <div>
        <h1 class="text-start m-0">{{ dataset.metadata.title }}</h1>
        <h3 class="text-start m-0 text-muted">{{ dataset.metadata.provider }}</h3>
        <p class="text-start m-0">{{ dataset.metadata.description }}</p>
      </div>
      <div class="row">
        <div class="col-5 d-flex align-items-center">
          <Icon icon="bxs:truck" style="font-size: 1.5rem" />
          <p class="m-0 text-nowrap">Trucks</p>
        </div>
        <div class="col-1 text-end">{{ dataset.numberOfTrucks }}</div>
        <div class="col-5 d-flex align-items-center">
          <Icon icon="bxs:gas-pump" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">Avg. fuel consumption</p>
        </div>
        <div class="col-1 text-end">{{ dataset.avgFuelConsumption }}</div>
      </div>
      <div class="row">
        <div class="col-5 d-flex align-items-center">
          <Icon icon="mdi:highway" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">Active</p>
        </div>
        <div class="col-1 text-end">{{ dataset.drivingTrucks }}</div>
        <div class="col-5 d-flex align-items-center">
          <Icon icon="mdi-light:sleep" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">Resting</p>
        </div>
        <div class="col-1 text-end">{{ dataset.restingTrucks }}</div>
      </div>
        </div>
        <div class="d-flex flex-column justify-content-center align-items-end">
          <p class="m-0" style="font-size: 3rem">{{ roundedFee }} € p.a.</p>
          <p class="m-0">Suggested insurance fee per truck</p>
        </div>
      </div>
      <div id="map" style="height: 600px"/>

      <div class="d-flex flex-column align-items-center justify-content-center">
        <table class="table table-hover">
          <thead>
            <tr>
              <th
                class="text-nowrap"
                v-for="(_, key) in dataset.data[0]"
                :key="key"
              >
                {{ key }}
              </th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="row in truckData" :key="row">
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
        <TablePagination :elements="elements" @pageChange="pageChanged"/>
      </div>
    </div>
  </div>
</template>

<script>
import { Icon } from '@iconify/vue'
import leaflet from 'leaflet'
import HeatmapOverlay from 'leaflet-heatmap'
import TablePagination from '@/components/TablePagination.vue'

export default {
  components: {
    Icon,
    TablePagination
  },
  data () {
    return {
      dataset: {
        data: [],
        metadata: {
          title: '',
          description: '',
          provider: ''
        },
        numberOfTrucks: 0,
        restingTrucks: 0,
        drivingTrucks: 0,
        avgFuelConsumption: 0,
        insuranceFee: 0
      },
      currentPage: 1,
      alert: false
    }
  },
  computed: {
    elements () {
      return this.dataset.data.length
    },
    truckData () {
      return this.dataset.data.slice((this.currentPage - 1) * 15, (this.currentPage - 1) * 15 + 15)
    },
    roundedFee () {
      return this.dataset.insuranceFee.toLocaleString('en-US', { maximumFractionDigits: 2, minimumFractionDigits: 2 })
    }
  },
  methods: {
    pageChanged (newPage) {
      this.currentPage = newPage
    }
  },
  async mounted () {
    if (this.$store.getters.getCurrentDataset === null) {
      await this.$store
        .dispatch('fetchDataset', this.$route.params.datasetId)
        .then((response) => {
          this.dataset = response.data
        })
        .catch(() => {
          this.alert = true
        })
    } else {
      this.dataset = this.$store.getters.getCurrentDataset
      this.$store.dispatch('setCurrentDataset', null)
    }
    const CENTER_COORDINATES_GERMANY = [51.1657, 10.4515]
    const map = leaflet.map('map').setView(CENTER_COORDINATES_GERMANY, 6)
    leaflet
      .tileLayer(
        'https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=pk.eyJ1IjoianVibGExMDIiLCJhIjoiY2wxZXMzZ3Z6MGx6YzNjbG45bGd1ZjV3ciJ9.QDU9GpeOQSK-zSCTb-lLTg',
        {
          attribution:
            'Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
          minZoom: 2,
          maxZoom: 18,
          id: 'mapbox/streets-v11',
          tileSize: 512,
          zoomOffset: -1,
          accessToken:
            'pk.eyJ1IjoianVibGExMDIiLCJhIjoiY2wxZXMzZ3Z6MGx6YzNjbG45bGd1ZjV3ciJ9.QDU9GpeOQSK-zSCTb-lLTg'
        }
      )
      .addTo(map)
    // leaflet.marker(CENTER_COORDINATES_GERMANY).addTo(map)
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
      latField: 'latitude',
      // which field name in your data represents the longitude - default "lng"
      lngField: 'longitude',
      // which field name in your data represents the data value - default "value"
      valueField: 'count'
    }

    const coordinates = {
      data: this.dataset.data
    }

    const heatmapLayer = new HeatmapOverlay(cfg)
    heatmapLayer.setData(coordinates)
    heatmapLayer.addTo(map)
  }
}
</script>

<style scoped>
.table > :not(:first-child) {
  border-top: 0;
}
</style>
