<template>
  <div class="d-flex justify-content-center gap-3">
    <div class="d-flex flex-column p-3 gap-3">
      <OfferForm :title="metadata.title" :description="metadata.description" :price="metadata.price" @titleChanged="titleChanged" @priceChanged="priceChanged" @descriptionChanged="descriptionChanged"/>
      <button type="submit" class="btn btn-primary fw-bold" style="width: 10rem" @click="createOffer()">Create offer</button>
    </div>
    <PolicyForm :startTime="metadata.policy.startTime" :endTime="metadata.policy.endTime" :expiresOn="metadata.policy.expiresOn" :maxUsages="metadata.policy.maxUsages" :localLogging="metadata.policy.localLogging" :remoteLogging="metadata.policy.remoteLogging"
      @start-time-changed="startTimeChanged" @end-time-changed="endTimeChanged" @expires-on-changed="expiresOnChanged" @max-usages-changed="maxUsagesChanged" @local-logging-changed="localLoggingChanged" @remote-logging-changed="remoteLoggingChanged"
    />
  </div>
</template>

<script>
import PolicyForm from '@/components/PolicyForm.vue'
import OfferForm from '@/components/OfferForm.vue'
import axios from 'axios'

export default {
  components: {
    PolicyForm,
    OfferForm
  },
  data () {
    return {
      metadata: {
        title: '',
        description: '',
        price: 0,
        policy: {
          startTime: '',
          endTime: '',
          expiresOn: '',
          maxUsages: 0,
          localLogging: false,
          remoteLogging: false
        }
      }
    }
  },
  methods: {
    createOffer () {
      let datarows = []
      const parts = this.$route.query.rows.split(';')
      parts.forEach(part => {
        if (part.includes('-')) {
          const start = parseInt(part.split('-')[0])
          const end = parseInt(part.split('-')[1])
          datarows = datarows.concat(this.$store.getters.getDatarows.slice(start - 1, end))
        } else {
          datarows.push(this.$store.getters.getDatarows[parseInt(part) - 1])
        }
      })
      axios.post('/offers', {
        metadata: this.metadata,
        data: datarows
      }).then(() => {
        this.$router.push('/offers')
      })
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
      this.metadata.policy.startTime = time
    },
    endTimeChanged (time) {
      this.metadata.policy.endTime = time
    },
    expiresOnChanged (date) {
      this.metadata.policy.expiresOn = date
    },
    maxUsagesChanged (usages) {
      this.metadata.policy.maxUsages = usages
    },
    localLoggingChanged (logging) {
      this.metadata.policy.localLogging = logging
    },
    remoteLoggingChanged (logging) {
      this.metadata.policy.remoteLogging = logging
    }
  }
}
</script>
