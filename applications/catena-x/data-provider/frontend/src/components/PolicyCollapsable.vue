<template>
  <div :id="'policies-' + id" class="collapse">
    <div class="d-flex flex-column gap-2">
      <div v-if="policy.maxUsages !== 0" class="d-flex justify-content-between">
        <Icon icon="ic:baseline-replay" class="icon-medium"/>
        <p class="m-0 text-nowrap">{{ policy.maxUsages }} {{ usages }}</p>
      </div>
      <div v-if="policy.startTime !== null || policy.endTime !== null" class="d-flex justify-content-between">
        <Icon icon="akar-icons:clock" class="icon-medium"/>
        <p class="m-0 text-nowrap">{{ times }}</p>
      </div>
      <div v-if="policy.expiresOn !== null" class="d-flex justify-content-between">
        <Icon icon="akar-icons:calendar" class="icon-medium"/>
        <p class="m-0 text-nowrap">{{ policy.expiresOn }}</p>
      </div>
      <div v-if="policy.localLogging || policy.remoteLogging" class="d-flex justify-content-between">
        <Icon icon="fe:document" class="icon-medium"/>
        <p class="m-0 text-nowrap">{{ logging }}</p>
      </div>
    </div>
  </div>
</template>

<script>
import { Icon } from '@iconify/vue'

export default {
  components: {
    Icon
  },
  props: {
    id: {
      type: String,
      required: true
    },
    policy: {
      type: Object,
      required: true
    }
  },
  computed: {
    usages () {
      return this.policy.maxUsages === 1 ? 'usage' : 'usages'
    },
    times () {
      if (this.policy.startTime !== null && this.policy.endTime !== null) {
        return this.policy.startTime.slice(0, -3) + ' - ' + this.policy.endTime.slice(0, -3)
      }

      if (this.policy.startTime !== null) {
        return 'from ' + this.policy.startTime.slice(0, -3)
      }

      return 'until ' + this.policy.endTime.slice(0, -3)
    },
    logging () {
      if (this.policy.localLogging && this.policy.remoteLogging) {
        return 'local, remote'
      }

      if (this.policy.localLogging) {
        return 'local'
      }

      return 'remote'
    }
  }
}
</script>

<style scoped>
.icon-medium {
  font-size: 1.25rem
}
</style>
