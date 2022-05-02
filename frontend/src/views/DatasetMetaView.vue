<template>
  <div class="d-flex justify-content-center gap-3">
    <div class="d-flex flex-column gap-3">
      <DatasetForm :title="metadata.title" :description="metadata.description" :price="metadata.price" @titleChanged="titleChanged" @priceChanged="priceChanged" @descriptionChanged="descriptionChanged"/>
      <button type="submit" class="btn btn-primary fw-bold" style="width: 10rem" @click="createDataset()">Create dataset</button>
    </div>
    <PolicyForm :startTime="policy.startTime" :endTime="policy.endTime" :expiresOn="policy.expiresOn" :numberOfUsages="policy.numberOfUsages" :localLogging="policy.localLogging" :remoteLogging="policy.remoteLogging"
      @start-time-changed="startTimeChanged" @end-time-changed="endTimeChanged" @expires-on-changed="expiresOnChanged" @number-of-usages-changed="numberOfUsagesChanged" @local-logging-changed="localLoggingChanged" @remote-logging-changed="remoteLoggingChanged"
    />
  </div>
</template>

<script>
import PolicyForm from '@/components/PolicyForm.vue'
import DatasetForm from '@/components/DatasetForm.vue'

export default {
  components: {
    PolicyForm,
    DatasetForm
  },
  data () {
    return {
      metadata: {
        title: '',
        description: '',
        price: 0
      },
      policy: {
        startTime: '',
        endTime: '',
        expiresOn: '',
        numberOfUsages: 0,
        localLogging: false,
        remoteLogging: false
      },
      rows: this.$route.params.file
    }
  },
  mounted () {
    console.log(this.$route.query.file)
  },
  methods: {
    createDataset () {
      console.log('create dataset')
      console.log(this.metadata)
      console.log(this.policy)
    },
    titleChanged (title) {
      this.metadata.title = title
    },
    descriptionChanged (description) {
      this.metadata.description = description
    },
    priceChanged (price) {
      this.metadata.price = price
    },
    startTimeChanged (time) {
      this.policy.startTime = time
    },
    endTimeChanged (time) {
      this.policy.endTime = time
    },
    expiresOnChanged (date) {
      this.policy.expiresOn = date
    },
    numberOfUsagesChanged (usages) {
      this.policy.numberOfUsages = usages
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
