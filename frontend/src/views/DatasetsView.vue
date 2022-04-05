<template>
  <div class="d-flex flex-wrap justify-content-center gap-4 mt-4">
    <Card :id="23421" description='Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet. Lorem ipsum dolor sit amet'/>
    <Card v-for="n in 10" :key="n" :id="n" description='Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod te'/>
  </div>
</template>

<script>
import Card from '@/components/DatasetCard.vue'
import 'bootstrap/js/dist/collapse'

export default {
  components: {
    Card
  }
}
</script>
