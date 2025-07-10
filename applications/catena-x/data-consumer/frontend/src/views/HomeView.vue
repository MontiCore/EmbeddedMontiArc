<template>
  <div class="home">
    <button class="btn btn-primary">Im a Btn</button>
    <Icon icon="fe:logout" />
    <img alt="Vue logo" src="../assets/logo.png" />
    <HelloWorld msg="Welcome to Your Vue.js App" />
    <Logo msg="Welcome to Your Vue.js App" />
    <h1>Hier drüber muss das logo sein</h1>
  </div>
</template>

<script>
// @ is an alias to /src
import HelloWorld from '@/components/HelloWorld.vue'
import Logo from '@/components/CompanyLogo.vue'
import { Icon } from '@iconify/vue'

export default {
  name: 'HomeView',
  components: {
    HelloWorld,
    Icon,
    Logo
  }
}
</script>
