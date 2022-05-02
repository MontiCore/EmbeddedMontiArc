<template>
  <div class="d-flex justify-content-center gap-3">
    <div class="d-flex flex-column gap-3">
      <DatasetForm :title="metaData.title" :description="metaData.description" :price="metaData.price" @titleChanged="titleChanged" @priceChanged="priceChanged" @descriptionChanged="descriptionChanged"/>
      <button type="submit" class="btn btn-primary fw-bold" style="width: 10rem" @click="createDataset()">Create dataset</button>
    </div>
    <PolicyForm :startTime="policy.startTime" :endTime="policy.endTime" :expiresOn="policy.expiresOn" :maxUsages="policy.maxUsages" :localLogging="policy.localLogging" :remoteLogging="policy.remoteLogging"
      @start-time-changed="startTimeChanged" @end-time-changed="endTimeChanged" @expires-on-changed="expiresOnChanged" @max-usages-changed="maxUsagesChanged" @local-logging-changed="localLoggingChanged" @remote-logging-changed="remoteLoggingChanged"
    />
  </div>
</template>

<script>
import PolicyForm from '@/components/PolicyForm.vue'
import DatasetForm from '@/components/DatasetForm.vue'
import axios from 'axios'
import { v4 as uuidv4 } from 'uuid'

export default {
  components: {
    PolicyForm,
    DatasetForm
  },
  data () {
    return {
      metaData: {
        title: '',
        description: '',
        price: 0
      },
      policy: {
        businessHours: {
          start: '',
          end: ''
        },
        expiresOn: '',
        maxUsages: 0,
        localLogging: false,
        remoteLogging: false
      }
    }
  },
  methods: {
    createDataset () {
      const datasetPolicy = this.policy
      datasetPolicy.id = uuidv4()
      datasetPolicy.event = 'data-access'
      axios.post('/dataset', {
        id: datasetPolicy.id,
        policy: datasetPolicy,
        metaData: this.metaData,
        file: this.$route.query.file,
        rows: this.$route.query.rows
      }).then(response => {
        console.log('success')
      })
    },
    titleChanged (title) {
      this.metaData.title = title
    },
    descriptionChanged (description) {
      this.metaData.description = description
    },
    priceChanged (price) {
      this.metaData.price = price
    },
    startTimeChanged (time) {
      this.policy.businessHours.start = time
    },
    endTimeChanged (time) {
      this.policy.businessHours.end = time
    },
    expiresOnChanged (date) {
      this.policy.expiresOn = date
    },
    maxUsagesChanged (usages) {
      this.policy.maxUsages = usages
    },
    localLoggingChanged (logging) {
      this.policy.localLogging = logging
    },
    remoteLoggingChanged (logging) {
      this.policy.remoteLogging = logging
    }
  }
}
</script>
