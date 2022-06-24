<template>
  <div class="header d-flex align-items-center">
    <div class="d-flex align-items-center carrier-logo" style="min-width: 20rem">
      <Logo/>
      <h1 class="m-0 fw-bold fst-italic">Carrier</h1>
    </div>
    <div class="d-flex justify-content-center flex-grow-1 gap-3">
        <router-link class="nav-item nav-link" to="/offers"><div :class="{ 'active-route': isCurrentView('datasets')}"><Icon class="pe-1" icon="bx:data" style="font-size: 1.75rem" />Datasets</div></router-link>
    </div>
    <button class="btn p-0 text-end" style="min-width: 20rem" type="button">
      <div style="color: #ffa630">
        <Icon icon="fe:logout" style="font-size: 2rem"/>
      </div>
    </button>
  </div>
</template>

<script>
import { Icon } from '@iconify/vue'
import Logo from '@/components/CompanyLogo.vue'

export default {
  components: {
    Icon,
    Logo
  },
  methods: {
    isCurrentView (route) {
      return this.$route.name === route
    }
  }
}
</script>

<style scoped>
.carrier-logo {
  color: #ffa630;
}

.active-route {
  color: #ffa630;
}

.nav-link {
  color: black;
  font-weight: bold;
}

.nav-link:hover, .nav-link:focus, .nav-link:active {
  color: #ffa630;
}

.header {
  border-bottom: solid;
  border-width: thin;
  border-color: lightgray;
  padding: 0.5rem;
}
</style>
