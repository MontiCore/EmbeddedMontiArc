<template>
  <div class="d-flex flex-column justify-content-center align-items-center">
    <img alt="Vue logo" src="../assets/logo.png">
    <PolicyForm />
    <HelloWorld msg="Welcome to Your Vue.js App"/>
  </div>
</template>

<script>
// @ is an alias to /src
import HelloWorld from '@/components/HelloWorld.vue'
import PolicyForm from '@/components/PolicyForm.vue'

export default {
  name: 'HomeView',
  components: {
    HelloWorld,
    PolicyForm
  }
}
</script>
